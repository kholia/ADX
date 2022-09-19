
/*
  Pi Pico W WiFi Station Demo
  picow-wifi-station.ino
  Use WiFi library to connect Pico W to WiFi in Station mode
 
  DroneBot Workshop 2022
  https://dronebotworkshop.com
*/
 
// Include the WiFi Library
#include <WiFi.h>
 
// Replace with your network credentials
const char* ssid = "Fibertel WiFi996 2.4GHz";
const char* password = "00413322447";
 
void setup() {
 
  // Start the Serial Monitor
  Serial.begin(115200);
 
  // Operate in WiFi Station mode
  WiFi.mode(WIFI_STA);
 
  // Start WiFi with supplied parameters
  WiFi.begin(ssid, password);
 
  // Print periods on monitor while establishing connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    delay(500);
  }
 
  // Connection established
  Serial.println("");
  Serial.print("Pico W is connected to WiFi network ");
  Serial.println(WiFi.SSID());
 
  // Print IP Address
  Serial.print("Assigned IP Address: ");
  Serial.println(WiFi.localIP());
 
}
 
void loop() {
 
  delay(2000);
 
  // Print IP Address
  Serial.print("Assigned IP Address: ");
  Serial.println(WiFi.localIP());
 
}
