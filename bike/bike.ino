/*
The StopPin should be replaced by the input pin from the wifi to stop the collection of data. 
StopPin = PD_7 (32) 

LCD Display 
Data pin: PD_1 (24)
Clk pin:  PD_0 (23) 
input pin from wifi module : PD_7 (32) 
Switch Button 1 starts the clock (timer) while 
input signal (pin 32) from wifi will stop the clock can also be Switch Button 2
 */
 
#include <Wire.h> 
#include <WiFi.h>
#include <SPI.h>

#define SA0 1
#if SA0
  #define MMA8452_ADDRESS 0x1D  // SA0 is high, 0x1C if low
#else
  #define MMA8452_ADDRESS 0x1C
#endif

char ssid[] = "SKWAD_NETWORK";
char password[] = "bmxrt2015";

IPAddress finishline(192, 168, 43, 119); 
WiFiClient bike;

boolean passedThreshold = false, start = false, gotReaction = false;
boolean timerOff = true, Connection = true, done = false; 
int i = 0, numPeaks = 0, maxPeak = 0, minPeak = 50000, peaks[251];
int counter = 0, accelCount[3], pastCount[3];
int startPin = PC_6, int1Pin = PB_0, int2Pin = PB_1;
double reaction_time; 
const byte SCALE = 2, dataRate = 0;

void setup() {
//#ifndef __AVR_ATtiny85__  
  Serial.begin(9600);
//#endif

  /////////////////////// Push Button ////////////////////////////
  pinMode(startPin, INPUT_PULLUP);
  pinMode(PUSH2, INPUT_PULLUP); 
  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, LOW);
  ////////////////////////////////////////////////////////////////

  /////////////////////// Accelorometer //////////////////////////
  Wire.setModule(0);
  Wire.begin();

  pinMode(int1Pin, INPUT);
  pinMode(int2Pin, INPUT);
  digitalWrite(int1Pin, LOW);
  digitalWrite(int2Pin, HIGH);
  
  if (readRegister(0x0D) == 0x2A) {  
    initMMA8452(SCALE, dataRate);
    Serial.println("MMA8452Q is online...");
  }
  
  else {
    Serial.print("Could not connect to MMA8452Q");
    while(true);
  }
  
  ////////////////////////////////////////////////////////////////
  
  ////////////////////////// Wi-Fi //////////////////////////////
  //Serial.print("1");

  SPI.setModule(0);
  //Serial.print("2");
  //WiFi.config(ip); 
  WiFi.begin(ssid, password);
  //Serial.println("3");

  // Wait until device is connected to network
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("Attempting to connect to network named: ");
    Serial.println(ssid); 
    delay(1500);
  }

  Serial.println("\nYou're connected to the network.");
  Serial.println("Waiting for local IP address.");
  
  while (WiFi.localIP() == INADDR_NONE) {
    Serial.print(".");
    delay(1000);
  }
  
  Serial.print("Local IP Address is: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  
  // Connect to Bike Unit on port 5000
  while ( !bike.connect(finishline, 5000) ) {
    Serial.println("Trying to connect to finish line.");
    delay(1000);
  }
  
  Serial.println("Connected to finish line server\n");
}

void sendToFinishLine(char data[], int size) {
  WiFiServer serverFinish(8888);
  serverFinish.begin();
  
  Serial.println("Starting bike server");
  while (!done) {
    WiFiClient finish = serverFinish.available();
    
    if (finish) {
      Serial.println("Connected");
      delay(10);
      for (i = 0; i < size; i++) {
        finish.write(data[i]);
        Serial.print(data[i]);
      }  
      Serial.println();
      done = true;
      delay(10);
      finish.stop();    
    }
  }
  
  Serial.println("Done sending");
  digitalWrite(RED_LED, LOW);
  delay(3000);
}

void convertToJson(double value, char key[], char output[]) {
  char converted[10];
  snprintf(converted, 10, "%.3f", value);
  strcat(output, key);
  strcat(output, converted);
  strcat(output, ",");
}

void convertTime(int value, char key[], char output[]) {
  char converted[10];
  snprintf(converted, 10, "%d", value);
  strcat(output, key);
  strcat(output, converted);
}

void prepareToSend(double ct, double rt, double _bpm, int peaks[], int size) {
  char allData[2000] = "";
  convertToJson(ct, "{\"completionTime\":", allData);
  convertToJson(rt, "\"reactionTime\":", allData);
  convertToJson(_bpm, "\"bpm\":",allData);
  
  strcat(allData, "\"heartRate\":[");

  for (i = 0; i < size; i++) {
    char output[10];
    sprintf(output, "%d", peaks[i]);
    strcat(allData, output);

    if (i != size-1)
        strcat(allData, ",");
  }

  strcat(allData, "]}\0");
  int new_size = strlen(allData);
  Serial.print("Size of data being sent ");
  Serial.println(new_size);
  sendToFinishLine(allData, new_size);
}

void readHeartRate() {
    int sensorValue = analogRead(A10); 

    if (sensorValue > 1000)
      passedThreshold = true;
      
      if (minPeak != 50000 && i < 250) {
        peaks[i++] = minPeak;
        minPeak = 50000;
      }
      
      if (sensorValue > maxPeak)
        maxPeak = sensorValue;
    
    else{
      if (passedThreshold){
         numPeaks++;
         passedThreshold = false; 

         
      if (maxPeak != 0 && i < 250) {
        peaks[i++] = maxPeak;
        maxPeak = 0;
      }
      
      if (sensorValue < minPeak)
        minPeak = sensorValue;
      }
    } 
}

void restart() {
  delay(1000);
  
  Serial.println("Resetting everything");
  passedThreshold = false; start = false; gotReaction = false;
  timerOff = true; Connection = true; done = false; 
  i = 0; numPeaks = 0; maxPeak = 0; minPeak = 50000;
  
  reaction_time = 0.0;
  digitalWrite(RED_LED, LOW); 
}
 
void startRecording() {
  Serial.println("Start Recording");
  unsigned long start1 = millis(), t;
  for (uint16_t s = 0; s < 9999; s++) { 
      t = millis() - start1;     
      Serial.print("1");
      if ((t % 50) == 0)
        readHeartRate();
      Serial.print("2");  
      if (bike.read() == 1) {
        Serial.println("Bike unit has passed the finish line");
        t = millis() - start1; 

        peaks[i] = '\0';
        double completionTime = (double) t /1000;
        double BPM = (numPeaks/(completionTime/60));
        
        prepareToSend(completionTime, reaction_time, BPM, peaks, i);
        
        restart();
        return;
      }
   }
}

void loop() { 
  digitalWrite(RED_LED, HIGH);
  Wire.setModule(0);
  Wire.begin();
    
  if (digitalRead(startPin)==LOW)
    start = true;
    
  if (start) {
    digitalWrite(RED_LED, LOW);
    start = false;

    Serial.println("User has started race");
    delay(5);
    bike.write(1);
    
    unsigned long start, finish; 
    start = millis(); 
    Serial.print("Start at ");
    Serial.println(start);
          
    do {
      if (digitalRead(int1Pin) == 1) {
        readAccelData(accelCount);

        if (counter > 3 && abs(accelCount[0] - pastCount[0]) > 2000) {
          if (!gotReaction) {
            finish = millis();
            gotReaction = true;   
          }
        }
             
        for (int i = 0; i < 3; i++)
          pastCount[i] = accelCount[i];
        
        counter++;
      }

    } while((millis()) - start < (7500));
     
    if (!gotReaction)
      finish = millis();
      
    Serial.print("Finish at ");
    Serial.println(finish);
    
    reaction_time = ((double)(finish - start)) / 1000; 
    digitalWrite(RED_LED, HIGH);
    startRecording();
  }
}



/////////////////////////////////// DONT TOUCH - FOR ACCELOROMETER /////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
void readAccelData(int * destination)
{
  byte rawData[6];  // x/y/z accel register data stored here

  readRegisters(0x01, 6, &rawData[0]);  // Read the six raw data registers into data array
  
  /* loop to calculate 12-bit ADC and g value for each axis */
  for (int i=0; i<6; i+=2)
  {
    destination[i/2] = ((rawData[i] << 8) | rawData[i+1]) >> 4;  // Turn the MSB and LSB into a 12-bit value
    if (rawData[i] > 0x7F)
    {  // If the number is negative, we have to make it so manually (no 12-bit data type)
      destination[i/2] = ~destination[i/2] + 1;
      destination[i/2] *= -1;  // Transform into negative 2's complement #
    }
  }
}

void initMMA8452(byte fsr, byte dataRate)
{
  MMA8452Standby();  // Must be in standby to change registers
  
  /* Set up the full scale range to 2, 4, or 8g. */
  if ((fsr==2)||(fsr==4)||(fsr==8))
    writeRegister(0x0E, fsr >> 2);  
  else
    writeRegister(0x0E, 0);
    
  /* Setup the 3 data rate bits, from 0 to 7 */
  writeRegister(0x2A, readRegister(0x2A) & ~(0x38));
  if (dataRate <= 7)
    writeRegister(0x2A, readRegister(0x2A) | (dataRate << 3));  
    
  /* Set up portrait/landscap registers - 4 steps:
     1. Enable P/L
     2. Set the back/front angle trigger points (z-lock)
     3. Set the threshold/hysteresis angle
     4. Set the debouce rate
  // For more info check out this app note: http://cache.freescale.com/files/sensors/doc/app_note/AN4068.pdf */
  writeRegister(0x11, 0x40);  // 1. Enable P/L
  writeRegister(0x13, 0x44);  // 2. 29deg z-lock (don't think this register is actually writable)
  writeRegister(0x14, 0x84);  // 3. 45deg thresh, 14deg hyst (don't think this register is writable either)
  writeRegister(0x12, 0x50);  // 4. debounce counter at 100ms (at 800 hz)
  
  /* Set up single and double tap - 5 steps:
     1. Set up single and/or double tap detection on each axis individually.
     2. Set the threshold - minimum required acceleration to cause a tap.
     3. Set the time limit - the maximum time that a tap can be above the threshold
     4. Set the pulse latency - the minimum required time between one pulse and the next
     5. Set the second pulse window - maximum allowed time between end of latency and start of second pulse
     for more info check out this app note: http://cache.freescale.com/files/sensors/doc/app_note/AN4072.pdf */
  writeRegister(0x21, 0x7F);  // 1. enable single/double taps on all axes
  // writeRegister(0x21, 0x55);  // 1. single taps only on all axes
  // writeRegister(0x21, 0x6A);  // 1. double taps only on all axes
  writeRegister(0x23, 0x20);  // 2. x thresh at 2g, multiply the value by 0.0625g/LSB to get the threshold
  writeRegister(0x24, 0x20);  // 2. y thresh at 2g, multiply the value by 0.0625g/LSB to get the threshold
  writeRegister(0x25, 0x08);  // 2. z thresh at .5g, multiply the value by 0.0625g/LSB to get the threshold
  writeRegister(0x26, 0x30);  // 3. 30ms time limit at 800Hz odr, this is very dependent on data rate, see the app note
  writeRegister(0x27, 0xA0);  // 4. 200ms (at 800Hz odr) between taps min, this also depends on the data rate
  writeRegister(0x28, 0xFF);  // 5. 318ms (max value) between taps max
  
  /* Set up interrupt 1 and 2 */
  writeRegister(0x2C, 0x02);  // Active high, push-pull interrupts
  writeRegister(0x2D, 0x19);  // DRDY, P/L and tap ints enabled
  writeRegister(0x2E, 0x01);  // DRDY on INT1, P/L and taps on INT2
  
  MMA8452Active();  // Set to active to start reading
}

/* Sets the MMA8452 to standby mode.
   It must be in standby to change most register settings */
void MMA8452Standby()
{
  byte c = readRegister(0x2A);
  writeRegister(0x2A, c & ~(0x01));
}

/* Sets the MMA8452 to active mode.
   Needs to be in this mode to output data */
void MMA8452Active()
{
  byte c = readRegister(0x2A);
  writeRegister(0x2A, c | 0x01);
}

/* Read i registers sequentially, starting at address 
   into the dest byte arra */
void readRegisters(byte address, int i, byte * dest)
{
  Wire.beginTransmission(MMA8452_ADDRESS); 
  
  Wire.write(address);
  Wire.endTransmission(false);
  
  Wire.requestFrom(MMA8452_ADDRESS, i);
 
  int j = 0;
  while(Wire.available())
  { 
    dest[j] = Wire.read();   
    j++;
  }
  Wire.endTransmission(); 
}

/* read a single byte from address and return it as a byte */
byte readRegister(uint8_t address)
{
  byte data;

  Wire.beginTransmission(MMA8452_ADDRESS); 
  
  Wire.write(address);
  Wire.endTransmission(false); 

  Wire.requestFrom(MMA8452_ADDRESS, 1);
  
  data = Wire.read();
  Wire.endTransmission();
    
  return data;
}

/* Writes a single byte (data) into address */
void writeRegister(unsigned char address, unsigned char data)
{
  Wire.beginTransmission(MMA8452_ADDRESS);
  Wire.write(address);
  Wire.write(data);  
  Wire.endTransmission();
}
