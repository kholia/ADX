#include <NTPClient.h>
// change next line to use with another board/shield
//#include <ESP8266WiFi.h>
#include <WiFi.h> // for WiFi shield
//#include <WiFi101.h> // for WiFi 101 shield or MKR1000
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

const char *ssid     = "Fibertel WiFi996 2.4GHz";
const char *password = "00413322447";

WiFiUDP ntpUDP;
//NTPClient timeClient(ntpUDP);
#define TZ -3600*3           //minutes
#define NTP_INTERVAL 60000   //mSecs

NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", TZ, NTP_INTERVAL);

//setTimeOffset()
//setUpdateInterval()


#include <time.h>
#include <stdlib.h>

uint32_t tnow=0;
uint32_t diff=0;
char hi[80];

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

void setup(){
  Serial.begin(115200);
  while (!Serial) {}
  delay(1);
  WiFi.begin(ssid, password);

  while ( WiFi.status() != WL_CONNECTED ) {
    delay ( 500 );
    Serial.print ( "." );
  }

   // Connection established
  Serial.println("");
  Serial.print("Pico W is connected to WiFi network ");
  Serial.println(WiFi.SSID());
 
  // Print IP Address
  Serial.print("Assigned IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Connecting to NTP Server");

  timeClient.begin();
  tnow = time_us_32();
  
}

void loop() {
  if (!timeClient.forceUpdate()) {
     Serial.println("Network update error");     
  }
  
  Serial.println(timeClient.getFormattedTime());
  uint32_t e=timeClient.getEpochTime();
  uint32_t t=time_us_32();
  
  int dow =(((e  / 86400L) + 4 ) % 7); //0 is Sunday
  int hour= ((e  % 86400L) / 3600);
  int min = ((e % 3600) / 60);
  int sec = (e % 60);

  sprintf(hi,"Epoch-> [%02d:%02d:%02d dow(%d)]\n",hour,min,sec,dow);
  Serial.print(hi);

  tnow=time_us_32();
  while (time_us_32() - tnow < 10000000) {}

  uint32_t f=e+((time_us_32()-tnow)/1000000);
  int fdow =(((f  / 86400L) + 4 ) % 7); //0 is Sunday
  int fhour= ((f  % 86400L) / 3600);
  int fmin = ((f % 3600) / 60);
  int fsec = (f % 60);

  sprintf(hi,"VRTC -> [%02d:%02d:%02d dow(%d)]\n",fhour,fmin,fsec,fdow);
  Serial.print(hi);


}
