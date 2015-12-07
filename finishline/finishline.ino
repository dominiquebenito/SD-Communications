// Note: connect one wire to GND other to P06
// IP Address: 192.168.43.119
// MAC address: B7:92:CB:5E:B8:F4
// NetMask: 255.255.255.0
// Gateway: 192.168.43.1
#include <WiFi.h>
#include <SPI.h>
#include <parse.h>
#include <RTClib.h>

RTC_Millis RTC;

// Use pin P06 for input signal for the finish line switch
int switchpad = 14;
int _bytes = 0;

// Network information on my phone
char ssid[] = "SKWAD_NETWORK";
char password[] = "bmxrt2015";
char allData[2000];

WiFiServer server(5000);
IPAddress ip(192, 168, 43, 119);  

void setup(){
  Serial.begin(9600);
  pinMode(14, INPUT_PULLUP);
  pinMode(31, OUTPUT);
  pinMode(11, OUTPUT);
  
  digitalWrite(31, HIGH);
  digitalWrite(11, LOW);
  
  // Connect to WPA network
  //WiFi.config(ip);
  WiFi.begin(ssid, password);
  
  // Wait until device is connected to network
  while ( WiFi.status() != WL_CONNECTED) {
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
  
  server.begin();
  
  RTC.begin(DateTime(__DATE__, __TIME__));
}

void restart() {
  Serial.println("Resetting");
  pinMode(14, INPUT_PULLUP);  
}

void readData(){
  Serial.print("");
  IPAddress bikeServer(192, 168, 43, 145); 
  WiFiClient bike;
  
  Serial.print("");
  while ( !bike.connect(bikeServer, 8888) );
  Serial.print("");
  Serial.println("Connected to bike server");
  
  int i = 0;
  while (bike.connected()) {
    if (bike.available()){
      allData[i] = bike.read();
      Serial.print(allData[i]);
      i++;
    }
  }
  
  allData[i] = '\0';
  Serial.println();
  Serial.println("Sending data to parse now.");

  // TODO: Set to current date/time (within an hour precision)
  DateTime now = RTC.now();
  
  SlDateTime_t dateTime;
  memset(&dateTime, 0, sizeof(dateTime));
  dateTime.sl_tm_year = now.year();
  dateTime.sl_tm_mon = now.month();
  dateTime.sl_tm_day = now.day();
  dateTime.sl_tm_hour = now.hour();
  sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION, SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME, sizeof(SlDateTime_t), (unsigned char *)&dateTime);

  Serial.print(dateTime.sl_tm_mon);
  Serial.print("/");
  Serial.print(dateTime.sl_tm_day);
  Serial.print("/");
  Serial.print(dateTime.sl_tm_year);
  Serial.print(" ");
  Serial.print(dateTime.sl_tm_hour);
  Serial.println();
  
  // TODO: Add Parse code here
  Serial.println("Initialize Parse Server");
  ParseClient parseClient = parseInitialize("ClSFTVZMGC8QccfrGPxTP0QLuayUm78iDnwY19HP", "8AQltrVaz4xdWZCUnVz6V1QtoLZrBBxstuc0rPMq");  
  parseSendRequest(parseClient, "POST", "/1/classes/dbenito", allData, NULL);
  Serial.println("Done");
  
  restart();
}

void loop(){
  WiFiClient bike = server.available();
  digitalWrite(31, HIGH);
  digitalWrite(11, LOW);
  
  if (bike) {  
    if (digitalRead(14) == LOW) {
      Serial.println("Bike has passed finish line");
      bike.write(1);
      Serial.print("");      
      readData();
    }
    
    if (bike.read() == 1) {
      Serial.println("User has started race");
      digitalWrite(31, LOW);
      digitalWrite(11, HIGH);
      delay(10000);
    }
  }
  
  delay(100);
}


