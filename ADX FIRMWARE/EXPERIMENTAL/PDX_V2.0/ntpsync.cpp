#include <Arduino.h>
#include "pdx_common.h"

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                          ntp CONFIGURATION                                                  *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*   
/*----------------------------------------------------------------------------------------------*
 * NTP_CLIENT implements a NTP (Network Time Protocol) client which can be used to optionally   *
 * sync the internal clock with an Internet based time server using a WiFi connection           *
 * The connection isn't constant but periodic, between connections the time is kept by the      *
 * internal Pico clock.                                                                         *
 *----------------------------------------------------------------------------------------------*/
#ifdef NTPSYNC

#define WIFI_TOUT                 60        //WiFi connection timeout (in Secs)
#define INET_TOUT                 20        //Inet access timeout (in Secs)
#define NTP_TOUT                  20        //NTP server time synchro timeout (in Secs)

#define NTP_SERVER1               "time.nist.gov"    //NTP server primary
#define NTP_SERVER2               "pool.ntp.org"     //NTP server secondary
#define INET_SERVER               "www.google.com"  //If Google isn't there, you're not connected

#define SYNC_OK                   0
#define SYNC_NO_INET              1
#define SYNC_NO_AP                2
#define SYNC_NO_NTP               3

/*------------------------
 * Define internet resources to be used
 */
const char* ntp_server1=NTP_SERVER1;
const char* ntp_server2=NTP_SERVER2;
const char* inet_server=INET_SERVER;

//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*                               WiFi Access Point credentials                                           *
//* Replace the AP SSID and password of your choice, if not modified the firmware won't be able to sync   *
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
const char* ssid = "WiFi AP SSID 2.4GHz";
const char* password = "0123456789";

/*------------------------
 * Take an IPAddress object and returns a String-ified version of it
 * for printing and display purposes
 */
String IpAddress2String(const IPAddress& ipAddress)
{
  return String(ipAddress[0]) + String(".") +\
  String(ipAddress[1]) + String(".") +\
  String(ipAddress[2]) + String(".") +\
  String(ipAddress[3])  ; 
}
/*-------------------------
 * setClock()
 * synchronize system clock with an NTP server using internet connectivity
 * return
 *    (true)        Synchronization was successful
 *    (false)       Synchronization failed
 */
bool setClock(char* n1,char* n2) {
  #ifdef DEBUG
    _INFOLIST("%s About to setClock (%s,%s)()\n",__func__,n1,n2);
  #endif //DEBUG

  NTP.begin(n1,n2);
  
  #ifdef DEBUG
      _INFOLIST("%s Waiting for NTP time sync with (%s,%s)\n",__func__,n1,n2);
  #endif //DEBUG
  bool ntp_rc=NTP.waitSet(NTP_TOUT*1000);
  #ifdef DEBUG
      _INFOLIST("%s NTP time sync finalized with status(%s)\n",__func__,BOOL2CHAR(ntp_rc));
  #endif //DEBUG

  if(ntp_rc) {
    return ntp_rc;    
  }
  time_t now = time(nullptr);
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  #ifdef DEBUG
     _INFOLIST("%s NTP time is %s\n",__func__,asctime(&timeinfo));
  #endif //DEBUG
  return ntp_rc;
}
/*------------
 *  checkInet()
 *  Check if there is actual internet connectivity by sending a successful ping to a known host
 */
int checkInet(char* hostName) {
  #ifdef DEBUG
     _INFOLIST("%s About to checkInet(%s) \n",__func__,hostName);
  #endif //DEBUG
  int pingResult = WiFi.ping(hostName);
  #ifdef DEBUG
     _INFOLIST("%s Ping host(%s) rtt=%d msec\n",__func__,hostName,pingResult);
  #endif //DEBUG
  return pingResult;
}
/*-------------
 *  checkAP()
 *  Check if there is an AP and try to connect with it
 */
int checkAP(char* s,char* p) {

  #ifdef DEBUG
     _INFOLIST("%s About to checkAP() id(%s) password(%s)\n",__func__,s,p);
  #endif //DEBUG

  if (WiFi.status()==WL_CONNECTED) {
     #ifdef DEBUG
        _INFOLIST("%s Already connected to ssid(%s) IP(%s)\n",__func__,s,IpAddress2String(WiFi.localIP()).c_str());
     #endif //DEBUG
     return WL_CONNECTED;
  }  
  #ifdef DEBUG
     _INFOLIST("%s About to begin\n",__func__);
  #endif //DEBUG

  WiFi.mode(WIFI_STA);
  WiFi.begin(s,p);
  uint32_t t=time_us_32();
  #ifdef DEBUG
     _INFOLIST("%s Connecting...\n",__func__);
  #endif //DEBUG

  while (WiFi.status()!=WL_CONNECTED) {
      if(time_us_32()-t > WIFI_TOUT*1000000) {
        #ifdef DEBUG
          _INFOLIST("%s Failed to connect to ssid(%s)\n",__func__,s);
        #endif //DEBUG
        return WiFi.status();
      }
  }
  #ifdef DEBUG
     _INFOLIST("%s Connected to ssid(%s) IP(%s)\n",s,IpAddress2String(WiFi.localIP()).c_str());
  #endif //DEBUG   
  return (int) WL_CONNECTED;
}
/*---------------------------------------------
 * syncTime()
 * Connect to an AP, verify navigation and sync time
 *    SYNC_OK       Synchronization Ok
 *    SYNC_NO_INET  Synchronization failed, no internet
 *    SYNC_NO_AP    Synchronization failed, no WiFi AP
 *    SYNC_NO_NTP   Synchronization failed, no NTP server
 */
int syncTime() {

  #ifdef DEBUG
     _INFOLIST("%s About to syncTime()\n",__func__);
  #endif //DEBUG

  if (checkAP((char*)ssid,(char*)password) != WL_CONNECTED) {
     return SYNC_NO_AP;     
  }

  if (checkInet((char*)inet_server) < 0) {
     return SYNC_NO_INET;
  } 

  if (setClock((char*)ntp_server1,(char*)ntp_server2)) {
     return SYNC_NO_NTP;
  }
  #ifdef DEBUG
     _INFOLIST("%s Clock synchronization completed successfully\n",__func__);
  #endif //DEBUG     
  return SYNC_OK;
}
#endif //NTPSYNC
