
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

#include <string.h>
#include <time.h>
#include <WiFi.h>                   // for WiFi AP connectivity
#include <WiFiUdp.h>

#define WIFI_TOUT        30
const char* ssid       = WIFI_SSID;
const char* password   = WIFI_PASW;
//const char *ssid     = "Fibertel WiFi996 2.4GHz";
//const char *password = "00413322447";
uint32_t   tout_WIFI     = 0;

/*---------------------------------------------------------
 * init_Wifi
 * The connection to a defined WiFi is attempted, the 
 * answer isn't verified here
 */
void init_WiFi() {

    #ifdef DEBUG
      _INFOLIST("@%s Starting connection ssid(%s)\n",__func__,ssid);
    #endif //DEBUG
    
    tout_WIFI=time_us_32();
    delay(500);
    WiFi.begin(ssid, password);
    while ( WiFi.status() != WL_CONNECTED && (time_us_32()-tout_WIFI)<10000000 ) {
       delay ( 500 );
    }
    tout_WIFI=time_us_32();
    setWord(&QSW,WIFIOK,false);

    #ifdef DEBUG
      _INFOLIST("@%s WiFi connection with ssid(%s) initiated\n",__func__,ssid);
    #endif //DEBUG

    
    /*
    #ifdef NTPC
       tout_NTPC=time_us_32();
    #endif //NTPC
    */
}
/*--------------------------------------------------------
 * check_Wifi
 * called from loop periodically to validate the connection
 * status and handle reconnection if necessary
 *--------------------------------------------------------*/
void check_WiFi() {

    if (WiFi.status() != WL_CONNECTED) {
       if (time_us_32()-tout_WIFI > WIFI_TOUT*1000000) {

          _INFOLIST("%s connecting again",__func__);
          WiFi.begin(ssid,password);
          tout_WIFI=time_us_32();
          setWord(&QSW,WIFIOK,false);      

          #ifdef DEBUG
             _INFOLIST("@%s WiFi resume connection with ssid(%s)\n",__func__,ssid);
          #endif //DEBUG
       }
    }

    if (WiFi.status() == WL_CONNECTED) {
       if (getWord(QSW,WIFIOK)==false) {
          setWord(&QSW,WIFIOK,true);
          #ifdef DEBUG
             sprintf(hi,"@%s WiFi connection established with ssid(%s) IP=",__func__,WiFi.SSID());
             _SERIAL.print(hi);
             _SERIAL.println(WiFi.localIP());
          #endif //DEBUG
       }
       tout_WIFI=time_us_32();
    }
}
#endif //WIFI
