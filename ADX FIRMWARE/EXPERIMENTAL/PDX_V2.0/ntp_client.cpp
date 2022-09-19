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


#ifdef NTP_CLIENT
time_t         t             = time(NULL);
struct tm      tms           = * localtime(&t);  

uint32_t       tout_NTPC     = 0;
uint32_t       vrtc_ntpc     = 0;
uint32_t       vrtc_snap     = 0;

WiFiUDP   ntpUDP;
NTPClient timeClient(ntpUDP,NTP_POOL, TZ, NTP_INTERVAL);

//setTimeOffset()
//setUpdateInterval()

#ifdef NTPC
          tout_NTPC=time_us_32();
#endif //NTPC    
#ifdef NTP_CLIENT
/*--------------------------------------------------------
 * check_NTP
 * if enabled sync the local clock with 
 *--------------------------------------------------------*/
void check_NTP() {
   
    if (WiFi.status() == WL_CONNECTED) {
       if (time_us_32()-tout_NTPC > NTPC_TOUT) {
          tout_NTPC=time_us_32();
          if (!timeClient.forceUpdate()) {
             #ifdef DEBUG
               _INFOLIST("%s NTP Server query failed\n",__func__);
             #endif //DEBUG
             return;
          }    

          vrtc_ntpc=timeClient.getEpochTime();        
          vrtc_snap=time_us_32();   
          
          #ifdef DEBUG
              _INFOLIST("%s NTP Server query success epoch(%ld) time(%s)\n",__func__,vrtc_ntpc,String(timeClient.getFormattedTime()));
          #endif //DEBUG
       }
    }
}

#endif //NTP_CLIENT
#endif //NTP_CLIENT

#endif //WIFI
