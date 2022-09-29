

#include <string.h>
#include <time.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "lwip/dns.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

#include <WiFi.h>

char hi[80];
const char* ssid = "Fibertel WiFi754 2.4GHz";
const char* password = "0041332244";
uint32_t tout_NTP=0;
bool fWIFI=false;
int16_t prevSec=0;
int pingResult;
const char* hostPing="www.google.com";



String IpAddress2String(const IPAddress& ipAddress)
{
  return String(ipAddress[0]) + String(".") +\
  String(ipAddress[1]) + String(".") +\
  String(ipAddress[2]) + String(".") +\
  String(ipAddress[3])  ; 
}
void printClock() {
  struct tm timeinfo;
  time_t now = time(nullptr);
  gmtime_r(&now, &timeinfo);
  sprintf(hi,"[VOID] Current time: %s\n",asctime(&timeinfo));
  Serial.print(hi);
  
}
void setClock() {
  NTP.begin("pool.ntp.org", "time.nist.gov");

  Serial.print("Waiting for NTP time sync: ");
  time_t now = time(nullptr);
  while (now < 8 * 3600 * 2) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println("");
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  sprintf(hi,"[SYNC] Current time: %s\n",asctime(&timeinfo));
  Serial.print(hi);
  //Serial.print("Current time: ");
  //Serial.print(asctime(&timeinfo));
}
void checkPing() {
  pingResult = WiFi.ping(hostPing);
  sprintf(hi,"[PING] host(%s) rtt=%d mSec\n",hostPing,pingResult);
  Serial.print(hi);

}
/*
void setClock() {
  NTP.begin("pool.ntp.org", "time.nist.gov");
  NTP.waitSet([]() { Serial.print("."); });
  time_t now = time(nullptr);
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  Serial.print("Current time: ");
  Serial.print(asctime(&timeinfo));
}
*/
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial);
  //const 
  //WiFi.begin(ssid,password);
  // Operate in WiFi Station mode
  WiFi.mode(WIFI_STA);
 
  // Start WiFi with supplied parameters
  //WiFi.begin(ssid, password);
  //WiFi.begin("Fibertel WiFi754 2.4GHz", "0041332244");
  WiFi.begin(ssid,password);

  while (WiFi.status()!=WL_CONNECTED);
  sprintf(hi,"Connected to ssid(%s) IP(%s)\n",ssid,IpAddress2String(WiFi.localIP()).c_str());
  Serial.print(hi);
  fWIFI=true;

  //NTP.begin("pool.ntp.org", "time.nist.gov");
  setClock();
  tout_NTP=time_us_32();
}

void loop() {
  // put your main code here, to run repeatedly:

  

  if (time_us_32()-tout_NTP > 30000000) {
     if (WiFi.status()!=WL_CONNECTED) {
        fWIFI=false;
        sprintf(hi,"Disconnected from ssid(%s), reconnecting\n",ssid);
        WiFi.disconnect();
        delay(100);
        Serial.print(hi);
        WiFi.begin(ssid,password);
     } else {
       if (fWIFI==false) {
          fWIFI=true;
          sprintf(hi,"Connected to ssid(%s) IP(%s)\n",ssid,IpAddress2String(WiFi.localIP()).c_str());
          Serial.print(hi); 
       }
       setClock();
       checkPing();
     }
     tout_NTP=time_us_32();
  } else {
    time_t now = time(nullptr);
    struct tm timeinfo;
    gmtime_r(&now, &timeinfo);
    
    if ( (timeinfo.tm_sec%10) == 0 && timeinfo.tm_sec != prevSec) { 
    //if ((time_us_32()-tout_NTP)%1000000 == 0) {
       printClock();    
       prevSec=timeinfo.tm_sec;
    }
  }
}
