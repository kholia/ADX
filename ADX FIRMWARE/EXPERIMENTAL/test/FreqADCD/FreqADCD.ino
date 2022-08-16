/**
 * ADC Frequency Counter
 * PDX project test code
 * Based on:
 * Copyright (c) 2021 Raspberry Pi (Trading) Ltd.
 * 
 *
 * SPDX-License-Identifier: BSD-3-Clause
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


/* Example code to extract analog values from a microphone using the ADC
   with accompanying Python file to plot these values
   Connections on Raspberry Pi Pico board, other boards may vary.
   GPIO 26/ADC0 (pin 31)-> AOUT or AUD on microphone board
   3.3v (pin 36) -> VCC on microphone board
   GND (pin 38)  -> GND on microphone board
*/

#define ADC_NUM 1
#define ADC_PIN (26 + ADC_NUM)
#define ADC_VREF 3.3
#define ADC_RANGE (1 << 12)
#define ADC_CONVERT (ADC_VREF / (ADC_RANGE - 1))

char hi[80];




void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial);
  Serial.println("FreqADCA Frequency counter and ADC evaluation program");

  rp2040.idleOtherCore();
  delay(1000);
  rp2040.restartCore1();
  delay(1); 

}

void setup1() {

  Serial.println("Core1 thread initialized");
  adc_init();
  adc_gpio_init( ADC_PIN);
  adc_select_input( ADC_NUM);

  uint adc_raw;
  uint adc_min=65535;
  uint adc_max=0;
  uint32_t sum_dt=0;
  uint32_t n=0;
  Serial.println("ADC converter initialized");
  while (n<1000) {
        uint32_t t1m=time_us_32();
        adc_raw = adc_read(); // raw voltage from ADC
        if (adc_raw>adc_max) adc_max=adc_raw;
        if (adc_raw<adc_min) adc_min=adc_raw;
        uint32_t t2m=time_us_32();
        sum_dt=sum_dt+t2m-t1m;
        n++;
        //sleep_ms(10);
  }
  
  uint adc_zero=uint((adc_max-adc_min)*1.0/2.0)+adc_min;
  Serial.printf("calibration n=%d Vmax=%d Vmin=%d Vmed=%d dt=%.2f\n",n,adc_max,adc_min,adc_zero,(1.0*sum_dt)/float(n));
  uint32_t tx1=0;
  uint32_t tx2=0;

  uint32_t t1[2];
  uint32_t t2[2];
  uint32_t v1[2];
  uint32_t v2[2];
  
  uint16_t adc_v1=0;
  uint16_t adc_v2=0;
  uint32_t adc_t1=0;
  uint32_t adc_t2=0;
  double   m=0.0;
  double   fmin=12000.0;
  double   fmax=0.0;
  double   ffsum=0.0;
  uint8_t  Q=0;

  

while (true) {

  switch(Q) {
    case 0 : {
               adc_v1=adc_read();
               adc_t1=time_us_32();
               uint32_t tstop=adc_t1+1000;
               while (true) {
                  adc_v2=adc_read();
                  adc_t2=time_us_32();
                  if (adc_v1 >= adc_zero && adc_v2 >= adc_zero) {
                     Q=1;
                     break;
                  }
                  adc_v1=adc_v2;
                  adc_t1=adc_t2;
                  if (time_us_32() > tstop) {Q=0;Serial.println("Timeout break Q=0");break;}
                 
               }                  
             }
    case 1 : {
               adc_v1=adc_read();
               adc_t1=time_us_32();
               uint32_t tstop=adc_t1+1000;

               while (true) {
                  adc_v2=adc_read();
                  adc_t2=time_us_32();
                  if (adc_v1 >= adc_zero && adc_v2 <= adc_zero) {
                     t1[0]=adc_t1;
                     t2[0]=adc_t2;
                     v1[0]=adc_v1;
                     v2[0]=adc_v2;
                     Q=2;
                     break;
                  }
                  
                  if (adc_v1 >= adc_zero && adc_v2 >= adc_zero) {
                     adc_v1=adc_v2;
                     adc_t1=adc_t2;
                     if (time_us_32() > tstop) {Q=0; Serial.println("Break Q=1");break;}
                     continue;
                  }
                  Serial.println("Invalid signal Q=1");
                  Q=0;
                  break;
                  

               }                  
                 
             }
    case 2 : {
               adc_v1=adc_read();
               adc_t1=time_us_32();
               uint32_t tstop=adc_t1+1000;

               while (true) {
                  adc_v2=adc_read();
                  adc_t2=time_us_32();
                  if (adc_v1 <= adc_zero && adc_v2 <= adc_zero) {
                     Q=3;
                     break;
                  }

                  if (adc_v1 >= adc_zero && adc_v2 <= adc_zero) {
                     t1[0]=adc_t1;
                     t2[0]=adc_t2;
                     v1[0]=adc_v1;
                     v2[0]=adc_v2;

                     adc_v1=adc_v2;
                     adc_t1=adc_t2;
                     if (time_us_32() > tstop) {Q=0; Serial.println("break Q=2");break;}
                     continue;
                  }
                  Serial.println("Invalid signal Q=2");
                  Q=0;
                  break;
               }
    } //Q=2
    case 3: {                     
               adc_v1=adc_read();
               adc_t1=time_us_32();
               uint32_t tstop=adc_t1+1000;

               while (true) {
                  adc_v2=adc_read();
                  adc_t2=time_us_32();
                  if (adc_v1 >= adc_zero && adc_v2 >= adc_zero) {
                     Q=4;
                     break;
                  }
                  adc_v1=adc_v2;
                  adc_t1=adc_t2;                  
                  if (time_us_32() > tstop) {Q=0; Serial.println("break Q=3");break;}

               }
    } //Q=3
    case 4: {
               adc_v1=adc_read();
               adc_t1=time_us_32();
               uint32_t tstop=adc_t1+1000;

               while (true) {
                  adc_v2=adc_read();
                  adc_t2=time_us_32();
                  if (adc_v1 >= adc_zero && adc_v2 <= adc_zero) {
                     t1[1]=adc_t1;
                     t2[1]=adc_t2;
                     v1[1]=adc_v1;
                     v2[1]=adc_v2;
                     Q=5;
                     break;
                  }
                  if (adc_v1 >= adc_zero && adc_v2 >=  adc_zero) {
                    adc_v1=adc_v2;
                    adc_t1=adc_t2;
                    if (time_us_32() > tstop) {Q=0; Serial.println("break Q=4");break;}
                    continue;
                  }
                  Serial.println("Invalid signal Q=4");
                  Q=0;
                  break;
               }
    } //Q=4

    case 5: {
               adc_v1=adc_read();
               adc_t1=time_us_32();
               uint32_t tstop=adc_t1+1000;

               while (true) {
                  adc_v2=adc_read();
                  adc_t2=time_us_32();
                  if (adc_v1 <= adc_zero && adc_v2 <= adc_zero) {
                     Q=6;
                     break;
                  }

                  if (adc_v1 >= adc_zero && adc_v2 <= adc_zero) {
                     t1[0]=adc_t1;
                     t2[0]=adc_t2;
                     v1[0]=adc_v1;
                     v2[0]=adc_v2;

                     adc_v1=adc_v2;
                     adc_t1=adc_t2;
                     if (time_us_32() > tstop) {Q=0; Serial.println("Break Q=5");break;}
                     continue;
                  }
                  Q=0;
                  Serial.println("Bad signal Q=5");
                  break;
                  }
    }     
    case 6: {

            //m=((adc_v2-adc_v1)*1.0/(adc_t2-adc_t1)*1.0);
            //tx1=adc_t1+uint32_t((adc_zero-adc_v1)*1.0/m);    

      
            double m=((v2[0]-v1[0])*1.0/(t2[0]-t1[0])*1.0);
            double t0s=t1[0]+uint32_t((adc_zero-v1[0])*1.0/m);    
            m=((v2[1]-v1[1])*1.0/(t2[1]-t1[1])*1.0);
            double t1s=t1[1]+uint32_t((adc_zero-v1[1])*1.0/m);    

            double f=1000000/(t1s-t0s);
            if (f<fmin) {fmin=f;}
            if (f>fmax) {fmax=f;}
            Serial.printf("f=%.2f Hz  fmin=%.2f fmax=%.2f\n",f,fmin,fmax);

            Q=7;
      
    }
    case 7: {
              sleep_ms(100);
              Q=0;
            }  

    
   //FSM(Q) Selector
}  //FSM(Q) Infinite looop
  
}
}
void loop() {
  // put your main code here, to run repeatedly:

}
void loop1() {}
