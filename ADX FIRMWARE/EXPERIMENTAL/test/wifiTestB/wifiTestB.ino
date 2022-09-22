// picoboy.ino
// 13th September 2022 Joe Brown.
// Basic socket server with static IP address.
// Wifi sections based on Earle's example WiFiServer credited below.
// Additions to wifi setup were to make IP address static. This follows
// my strategy  discovered for WEMOS D1 mini (ESP8266).
// There may be a better way, but I don't know it.
// 'WiFiServer' Placed in the public domain by Earle F. Philhower, III, 2022
// Example php and javascript code is appended as comments at end.

#include <WiFi.h>
#ifndef STASSID
#define STASSID "Wat2Much"
#define STAPSK "N0t3NuF"
#define _USE_SERIAL 
#endif
const char* ssid = STASSID;
const char* password = STAPSK;
int port = 29032; // base-36 'MEG'

WiFiServer server(port);
// client cmd reception buffer and count
uint8_t buf[32];
int count = 0;

// I don't like using delay(), preferring to yield whilst waiting for something
unsigned long lastmilli = 0;
const unsigned long del1000 = 1000;
const unsigned long del200 = 200;
const unsigned long del10 = 10;
void setup() 
{
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  WiFi.begin(ssid, password);
  WiFi.mode(WIFI_STA);
  #ifdef _USE_SERIAL
    Serial.printf("Connecting to '%s' with '%s'\n", ssid, password);
  #endif
  while (WiFi.status() != WL_CONNECTED) 
  {
    // delay with yield
    lastmilli = millis();
    while ((millis() - lastmilli) < del1000)
        yield();
    #ifdef _USE_SERIAL
      Serial.println("Connecting..");
    #endif
  }
  // Now disconnect, setup static IP and re-connect
  WiFi.disconnect();
  IPAddress ip(192,168,1,75); 
  IPAddress gateway(192,168,1,1);   
  IPAddress subnet(255,255,255,0); 
  WiFi.config(ip, gateway, subnet);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) 
  {
    #ifdef _USE_SERIAL
      Serial.print(".");
    #endif
    lastmilli = millis();
    while ((millis() - lastmilli) < del1000)
        yield();
  }
  #ifdef _USE_SERIAL
    Serial.printf("\nConnected to WiFi\n\nConnected to server at %s:%d\n", WiFi.localIP().toString().c_str(), port);
  #endif
  server.begin();
}

void heartbeat() 
{
  static byte heartState = 1;
  static unsigned long previousBeat = millis();
  unsigned long currentMillis;
  
  currentMillis = millis();
  if (currentMillis - previousBeat >= 500)
  {
    previousBeat = currentMillis;
    heartState ^= 1;
    digitalWrite(LED_BUILTIN, heartState); // on board LED
    //do any more timed jobs here
   }
}

void loop() 
{
  //delay 200ms
  lastmilli = millis();
  while ((millis() - lastmilli) < del200)
     yield();
     
  heartbeat(); // show I'm alive..
     
  WiFiClient client = server.available();
  if (!client) 
  {
    return;
  }
  
  // My client (HTML5/Javascript) uses a php gateway on my local intranet
  // to forward commands and requests.
  // These appear here as (in order) 'C', <cmd count>, cmd, [param1, param2 ...]
  // as you can see the minimum data count is 3, which would be a cmd without parameters
  // Note that a reply to the client must usually be sent, even if it's 1 byte only.
  if (client.available() > 0) // there is data?
  {
    while (client.available() > 0) // whilst there is data, read it into buffer
    {
      uint8_t c = client.read();
      buf[count++] = c;
      #ifdef _USE_SERIAL
        Serial.print(c, HEX); // debug printout
        Serial.print(", ");
      #endif
    }
    #ifdef _USE_SERIAL
      Serial.println();
    #endif
    if ( count >= 3 && count == (buf[1] + 2)) // cmd complete?
    {
        client.write(buf[3]); // return 1st parameter in this example
    }
    else
      client.write('\n');
    count = 0;
    client.flush();
  }
  
}
