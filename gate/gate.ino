#include <WiFi.h>
#include<SPI.h>

// Network information on my phone
char ssid[] = "SKWAD_NETWORK";
char password[] = "bmxrt2015";

WiFiClient client;
IPAddress server(192, 168, 43, 145);
IPAddress ip(192, 168, 43, 162);  

void setup(){
  Serial.begin(9600);
  pinMode(31, OUTPUT);
  digitalWrite(31, HIGH);
  
  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, LOW);
  
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
  
  // Connect to Bike Unit on port 5000
  while ( !client.connect(server, 5001) ) {
    Serial.println("Trying to connect to bike.");
    delay(1000);
  }
  
  Serial.println("Connected to bike server\n");
}

void loop(){
  if (client.read() == 1) {
    Serial.println("Sending byte to start cadence and lights");
    digitalWrite(31, LOW);
    digitalWrite(RED_LED, HIGH);
    delay(6500);
    Serial.println("Dropping Gate");
    digitalWrite(31, HIGH);
    digitalWrite(RED_LED, LOW);
    //client.stop();
    //while (true);
  }
}

