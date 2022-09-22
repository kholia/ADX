
/*============================================================================================
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Largely based on
 * Pi Pico W WiFi Station Demo
 * picow-wifi-station.ino
 * Use WiFi library to connect Pico W to WiFi in Station mode
 *
 * DroneBot Workshop 2022
 * https://dronebotworkshop.com
 * 
 * Modifications and adaptations to PDX project by Dr. Pedro E. Colla (LU7DZ) 2022
 *================================================================================================*/

#include <Arduino.h>
#include "pdx_common.h"

#ifdef WIFI

/*--------------------------------------------------------------------------------*
 * The TCP/IP connectivity is skip altogether if WiFi credentials aren't provided *
 * without them the program will be retrying needlessy into a non-existing AP with*
 * fake or no credentials
 */
#if __has_include("wifi_credentials.h")
#include "wifi_credentials.h"      //Include WiFi credentials
#else
#undef WIFI                        //Clear TCP/IP connectivity if credentials aren't provided
#endif
#endif //WIFI


#ifdef WIFI
#include <NTPClient.h>
#include <WiFi.h> // for WiFi shield
#include <WiFiUdp.h>


#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "ctime"

extern "C" {
  #include "hardware/rtc.h"
  #include "pico/util/datetime.h"
}

char datetime_buf[256];
char *datetime_str = &datetime_buf[0];
datetime_t t;

//const char *ssid     = "Fibertel WiFi996 2.4GHz";
//const char *password = "00413322447";

const char *ssid     = "Fibertel WiFi754 2.4GHz";
const char *password = "0041332244";

WiFiUDP ntpUDP;
#define TZ -3600*3           //minutes
#define NTP_INTERVAL 60000   //mSecs
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", TZ, NTP_INTERVAL);

//setTimeOffset()
//setUpdateInterval()


#include <time.h>
#include <stdlib.h>

uint32_t tnow=0;
uint32_t diff=0;


//void getdatetime(int &year, int &mon, int &day, int &hour, int &min, int &sec, int &dow) {
void getdatetime() {

   time_t t = time(NULL);
   struct tm  tms = * localtime(&t);  
   char hi[80];
   int year = tms.tm_year+1900; 
   int mon  = tms.tm_mon+1;
   int day  = tms.tm_mday;
   int hour = tms.tm_hour;
   int min  = tms.tm_min;
   int sec  = tms.tm_sec;
   int dow  = tms.tm_wday+1;      

   sprintf(hi,"%d/%d/%d %d:%d:%d dow=%d\n",year,mon,day,hour,min,sec,dow);
   Serial.print(hi);
}

void init_WiFi(){

  sprintf(hi,"Starting WiFi SSID(%s) PSK(%s)",ssid,password);
  _SERIAL.print(hi);
  
  WiFi.begin(ssid, password);
  
  WiFi.mode(WIFI_STA);
  _SERIAL.printf("Connecting to '%s' with '%s'\n", ssid, password);
  while (WiFi.status() != WL_CONNECTED) 
  {
    // delay with yield
    unsigned long lastmilli = millis();
    while ((millis() - lastmilli) < 1000)
      yield();
      _SERIAL.println("Connecting..");
  }

   // Connection established
  _SERIAL.println("");
  _SERIAL.print("Pico W is connected to WiFi network ");
  _SERIAL.println(WiFi.SSID());
 
  // Print IP Address
  _SERIAL.print("Assigned IP Address: ");
  _SERIAL.println(WiFi.localIP());
  _SERIAL.println("Connecting to NTP Server");

  timeClient.begin();
  tnow = time_us_32();
  
}

void check_WiFi() {
  if (!timeClient.forceUpdate()) {
     _SERIAL.println("Network update error");     
  }
  _SERIAL.println("Checking NTP server");
  _SERIAL.println(timeClient.getFormattedTime());
  uint32_t e=timeClient.getEpochTime();
  uint32_t t=time_us_32();
  
  int dow =(((e  / 86400L) + 4 ) % 7); //0 is Sunday
  int hour= ((e  % 86400L) / 3600);
  int min = ((e % 3600) / 60);
  int sec = (e % 60);

  sprintf(hi,"Epoch-> [%02d:%02d:%02d dow(%d)]\n",hour,min,sec,dow);
  _SERIAL.print(hi);

  tnow=time_us_32();
  while (time_us_32() - tnow < 10000000) {}

  uint32_t f=e+((time_us_32()-tnow)/1000000);
  int fdow =(((f  / 86400L) + 4 ) % 7); //0 is Sunday
  int fhour= ((f  % 86400L) / 3600);
  int fmin = ((f % 3600) / 60);
  int fsec = (f % 60);

  sprintf(hi,"VRTC -> [%02d:%02d:%02d dow(%d)]\n",fhour,fmin,fsec,fdow);
  _SERIAL.print(hi);


}
#endif //WIFI
