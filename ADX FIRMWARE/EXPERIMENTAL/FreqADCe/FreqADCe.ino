/**
 * ADC FreQSTATEuency Counter
 * PDX project test code
 * Based on:
 * Copyright (c) 2021 Raspberry Pi (Trading) Ltd.
 * 
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * Dr. P.E.Colla (LU7DZ) 2022
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
#include "pico/binary_info.h"

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"
#include <stdio.h>
#include "hardware/watchdog.h"
#include "hardware/pwm.h"
#include "pico/multicore.h"  


/* Extract analog values from the selected ADC pin
 * The signal might have any level of DC and might have some noise
 * so a strict finite state machine (FSM) is used to ensure that only
 * valid states are included in the frequency computation
 * Still, small measurement errors might lead to the wrong computation of 
 * the frequency.
*/

/*------------------------------*
 * ADC port values              *
 *------------------------------*/
#define ADC_NUM 1
#define ADC_PIN (26 + ADC_NUM)
#define ADC_VREF 3.3
#define ADC_RANGE (1 << 12)
#define ADC_CONVERT (ADC_VREF / (ADC_RANGE - 1))

/*------------------------------*
 * ADC Limits and operation     *
 *------------------------------*/
#define  ADCMAX      4096
#define  ADCMIN         0
#define  ADCZERO     (ADCMAX+ADCMIN)/2
#define  FSKMIN       200
#define  FSKMAX      3000
#define  ADCSAMPLE    100

/*------------------------------*
 * Epoch values
 *------------------------------*/
uint32_t t1[2];
uint32_t t2[2];
uint32_t v1[2];
uint32_t v2[2];


/*-------------------------------*  
 * Sample data 
 */
uint16_t adc_v1=0;
uint16_t adc_v2=0;
uint32_t adc_t1=0;
uint32_t adc_t2=0;

bool     adc_high=false;
bool     adc_low=false;

/*-------------------------------*
 * Computed frequency limits     *
 */
double   ffmin=FSKMAX;
double   ffmax=FSKMIN;
uint16_t adc_min=ADCMAX;
uint16_t adc_max=ADCMIN;
uint16_t adc_zero=ADCZERO;
uint16_t adc_uh;
uint16_t adc_ul;
/*--------------------------------*
 * FSM state variable
 *--------------------------------*/
uint8_t  QSTATE=0;


/*------------------------------------------------------------------------------------------*
 * calibrateADC
 * Calibrate the ADC zero level
 */
uint16_t calibrateADC(uint16_t min,uint16_t max) {
     return uint16_t((adc_max-adc_min)*1.0/2.0)+adc_min;   
}
/*-------------------------------------------------------------------------------------------*
 * ADCreset
 * restore all calibration values
 * 
 */
void ADCreset() {
     adc_min=ADCMAX;
     adc_max=ADCMIN;
     adc_zero=ADCZERO;                
     adc_uh=adc_zero*110/100;
     adc_ul=adc_zero*90/100;
     ffmin=FSKMAX;
     ffmax=FSKMIN;
     Serial.println("Timeout break QSTATE=0, recalibrate input level");
}
/*------------------------------------------------------------------------------------------*
 * getADCsample
 * collect an ADC sample running free.
 * update the minimum and maximum
 */
uint16_t getADCsample() {
  uint16_t v=adc_read();
  if (v>adc_max) {
     adc_max=v;
     adc_zero=calibrateADC(adc_min,adc_max);
     adc_uh=adc_zero*110/100;
     adc_ul=adc_zero*90/100;
     Serial.printf("calibration adc_max=%d adc_min=%d adc_Zero=%d\n",adc_max,adc_min,adc_zero);

  }
  if (v<=adc_min) {
     adc_min=v;
     adc_zero=calibrateADC(adc_min,adc_max);
     adc_uh=adc_zero*110/100;
     adc_ul=adc_zero*90/100;
     Serial.printf("calibration adc_max=%d adc_min=%d adc_Zero=%d\n",adc_max,adc_min,adc_zero);
  }
  if (v>=adc_uh) {adc_high=true;}
  if (v<=adc_ul) {adc_low =true;} 
  
  return  v; 
}
/*-------------------------------------------------------*
 * setup 
 * main initialization
 */
void setup() {


  /*--------------------------------------*
   * Initialize Serial port
   *--------------------------------------*/
  Serial.begin(115200);
  while(!Serial);
  Serial.println("FreqADCe Frequency counter and ADC evaluation program");

  /*--------------------------------------*
   * Launch core1
   */
  rp2040.idleOtherCore();
  delay(1000);
  rp2040.restartCore1();
  delay(1); 

}

/*--------------------------------------------------------*
 * setup of core1 initialization thread
 * in this demo program this procedure is an infinite loop
 */
void setup1() {

  Serial.println("Core1 thread initialized");
  /*------------------------------------*
   * ADC initialization and setup
   */
  adc_init();
  adc_gpio_init( ADC_PIN);
  adc_select_input( ADC_NUM);

  /*---------------------------------------------------*
   * Signal processing
   * This processing is heavy filtered of inconsistent
   * states produced by noise in the signal
   */

while (true) {

  switch(QSTATE) {
    //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
    //* State 0 - Wait for the signal to be positive to start a counting cycle       *
    //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
    case 0 : {
               adc_v1=getADCsample();               //Wait till two sucessive readings are positive, exit if none in 1 mSec
               adc_t1=time_us_32();
               uint32_t tstop=adc_t1+1000;
               adc_high=false;
               adc_low=false;
               while (true) {
                  adc_v2=getADCsample();
                  adc_t2=time_us_32();
                  if (adc_v1 >= adc_zero && adc_v2 >= adc_zero) {
                     QSTATE=1;
                     break;
                  }
                  adc_v1=adc_v2;
                  adc_t1=adc_t2;
                  if (time_us_32() > tstop) {
                    QSTATE=0;
                    ADCreset();
                    break;
                  }
                 
               }                  
             }
    //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
    //* State 1 - Wait for a zero crossing to get the first epoch                    *
    //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
    case 1 : {
               adc_v1=getADCsample();               //Last state was at least one pair of positive values, so wait for a crossing
               adc_t1=time_us_32();                 //accept but ignore sucessive pairs of positive values, the state looks
               uint32_t tstop=adc_t1+1000;          //for one positive and the next negative. Any other combination reset the finite state machine

               while (true) {
                  adc_v2=getADCsample();
                  adc_t2=time_us_32();
                  if (adc_v1 >= adc_zero && adc_v2 <= adc_zero) {
                     t1[0]=adc_t1;
                     t2[0]=adc_t2;
                     v1[0]=adc_v1;
                     v2[0]=adc_v2;
                     QSTATE=2;
                     break;
                  }
                  
                  if (adc_v1 >= adc_zero && adc_v2 >= adc_zero) {
                     adc_v1=adc_v2;
                     adc_t1=adc_t2;
                     if (time_us_32() > tstop) {
                        QSTATE=0;
                        Serial.println("Break QSTATE=1");
                        break;
                     }
                     continue;
                  }
                  Serial.println("Bad signal QSTATE=1");
                  QSTATE=0;
                  break;
                  

               }                  
                 
             }
    //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
    //* State 2 - Wait for the signal to become fully negative                       *
    //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
    case 2 : {                                                //A cross between positive and negative was detected, if another crossing is detected
               adc_v1=getADCsample();                         //then sampling was fast enough to capture another sucessive crossing, thus the mark
               adc_t1=time_us_32();                           //is updated. If two sucessive negative values are detected the crossing is completed 
               uint32_t tstop=adc_t1+1000;                    //and the FSM is advanced to next state. Any other combination is weird and reset the FSM

               while (true) {
                  adc_v2=getADCsample();
                  adc_t2=time_us_32();
                  if (adc_v1 <= adc_zero && adc_v2 <= adc_zero) {
                     QSTATE=3;
                     break;
                  }

                  if (adc_v1 >= adc_zero && adc_v2 <= adc_zero) {
                     t1[0]=adc_t1;
                     t2[0]=adc_t2;
                     v1[0]=adc_v1;
                     v2[0]=adc_v2;

                     adc_v1=adc_v2;
                     adc_t1=adc_t2;
                     if (time_us_32() > tstop) {
                         QSTATE=0; 
                         Serial.println("break QSTATE=2");
                         break;
                     }
                     continue;
                  }
                  Serial.println("Bad signal QSTATE=2");
                  QSTATE=0;
                  break;
               }
    } //QSTATE=2
    //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
    //* State 3 - Wait for the signal to become fully positive, start 2nd checkpoint *
    //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
    case 3: {                                                  //At this point at least two successive negative values has been obtained
               adc_v1=getADCsample();                          //the sample stream is now evaluated and any value other than two sucessive positive values
               adc_t1=time_us_32();                            //is ignored. So when the signal swing back to positive the FSM is advanced.
               uint32_t tstop=adc_t1+1000;

               while (true) {
                  adc_v2=adc_read();
                  adc_t2=time_us_32();
                  if (adc_v1 >= adc_zero && adc_v2 >= adc_zero) {
                     QSTATE=4;
                     break;
                  }
                  adc_v1=adc_v2;
                  adc_t1=adc_t2;                  
                  if (time_us_32() > tstop) {
                     QSTATE=0;
                     Serial.println("break QSTATE=3");
                     break;
                  }

               }
    } //QSTATE=3
    //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
    //* State 4 - Wait for the 2nd zero crossing and take the epoch when detected    *
    //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
    case 4: {                                                  //At this point at least two sucessive positive values has been detected. Others might follow
               adc_v1=getADCsample();                          //which are ignored until another crossing is detected. This is similar to state <1> but for
               adc_t1=time_us_32();                            //the next cycle. Samples other than both positive or a crossing reset the FSM
               uint32_t tstop=adc_t1+1000;

               while (true) {
                  adc_v2=getADCsample();
                  adc_t2=time_us_32();
                  if (adc_v1 >= adc_zero && adc_v2 <= adc_zero) {
                     t1[1]=adc_t1;
                     t2[1]=adc_t2;
                     v1[1]=adc_v1;
                     v2[1]=adc_v2;
                     QSTATE=5;
                     break;
                  }
                  if (adc_v1 >= adc_zero && adc_v2 >=  adc_zero) {
                    adc_v1=adc_v2;
                    adc_t1=adc_t2;
                    if (time_us_32() > tstop) {
                       QSTATE=0;
                       Serial.println("break QSTATE=4");
                       break;
                    }
                    continue;
                  }
                  Serial.println("Bad signal QSTATE=4");
                  QSTATE=0;
                  break;
               }
    } //QSTATE=4

    //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
    //* State 5 - Wait for the signal to stabilize at negateive values               *
    //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
    case 5: {                                                     //At this point a crossing has been detected, the sample stream is inspected looking for another
               adc_v1=getADCsample();                             //crossing and values are updated. When two sucessive samples are negative the FSM is advanced
               adc_t1=time_us_32();                               //to status 6.
               uint32_t tstop=adc_t1+1000;

               while (true) {
                  adc_v2=getADCsample();
                  adc_t2=time_us_32();
                  if (adc_v1 <= adc_zero && adc_v2 <= adc_zero) {
                     QSTATE=6;
                     break;
                  }

                  if (adc_v1 >= adc_zero && adc_v2 <= adc_zero) {
                     t1[0]=adc_t1;
                     t2[0]=adc_t2;
                     v1[0]=adc_v1;
                     v2[0]=adc_v2;

                     adc_v1=adc_v2;
                     adc_t1=adc_t2;
                     if (time_us_32() > tstop) {
                        QSTATE=0; 
                        Serial.println("Break QSTATE=5");
                        break;
                     }
                     continue;
                  }
                  QSTATE=0;
                  Serial.println("Bad signal QSTATE=5");
                  break;
                  }
    }     
    //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
    //* State 6 - Two epoch available, compute the frequency                         *
    //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
    case 6: {                                                                    //Datum for two sucessive crossings has been collected at this point
                                                                                 //then a more precise computation of the exact epoch of each crossing      
            double m=((v2[0]-v1[0])*1.0/(t2[0]-t1[0])*1.0);                      //is performed. The frequency is computed as the projection of the difference
            double t0s=t1[0]+uint32_t((adc_zero-v1[0])*1.0/m);                   //between two sucessive crossings projected to a full second.
            m=((v2[1]-v1[1])*1.0/(t2[1]-t1[1])*1.0);
            double t1s=t1[1]+uint32_t((adc_zero-v1[1])*1.0/m);    
            /*---------------------------------------------------------*
             * This operates as a squelch by insuring that during the  *
             * frequency measurement both positive and negative values *
             * went in excess of the calibrated zero level +/-10%      *
             * and not computing the frequency if the signal was too   *
             * low because it would create large errors if so          *
             *---------------------------------------------------------*/
            if (adc_low == false || adc_high == false) {
               Serial.printf("Input signal too low to compute\n");
               ADCreset();
               QSTATE=7;
               break;
            }   
            if ((t1s>t0s)) {
              double f=1000000/(t1s-t0s);
              if (f>=double(FSKMIN) && f<=double(FSKMAX)) {
                 if (f<ffmin) {ffmin=f;}
                 if (f>ffmax) {ffmax=f;}
                 Serial.printf("f=%.2f Hz  fmin=%.2f fmax=%.2f\n",f,ffmin,ffmax);
                 /*-----------------------------*
                  * this is a valid f epoch     *       
                  *-----------------------------*/
                  
              } else {
                 Serial.printf("f=%.2f Hz  outside of frequency limits defined\n",f);                          
              }
            } else {
              Serial.printf("Invalid frequency marking is ignored\n");             
            }
            QSTATE=7;
      
    }
    //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
    //* State 7 - Wait for the next measurement                                      *
    //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
    case 7: {                                                                     //In this state the FSM rest for a given delay
              sleep_ms(ADCSAMPLE);
              QSTATE=0;
            }  

  }  //FSM(QSTATE) logic

} //FSQSTATE(QSTATE) infinite loop

}
void loop() {
  // This code is left executing in loop at core0, all computations are made at core1

}
void loop1() {}  //loop1 is core1 loop function but never gets executed as setup1 is an infinite loop.
