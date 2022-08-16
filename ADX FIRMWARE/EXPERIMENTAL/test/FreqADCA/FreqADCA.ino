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
  uint32_t t1=0;
  uint32_t t2=0;
  uint adc_min=65535;
  uint adc_max=0;
  uint32_t sum_dt=0;
  uint32_t n=0;
  Serial.println("ADC converter initialized");
  while (n<1000) {
        uint32_t t1=time_us_32();
        adc_raw = adc_read(); // raw voltage from ADC
        if (adc_raw>adc_max) adc_max=adc_raw;
        if (adc_raw<adc_min) adc_min=adc_raw;
        uint32_t t2=time_us_32();
        sum_dt=sum_dt+t2-t1;
        n++;
        //if (n==1000) {
           //Serial.printf("Raw=%d Sig=%.2f min=%d max=%d t2=%ld t1=%ld dt=%d [uSec]\n", adc_raw, adc_raw * ADC_CONVERT,adc_min,adc_max,t2,t1,(t2-t1));
           //Serial.printf("Avg sample=%.2f min=%d max=%d\n",(1.0*sum_dt)/float(n),adc_min,adc_max);
           //n=0;
           //sum_dt=0;
        //}
        //sleep_ms(10);
  }
  
  uint adc_zero=uint((adc_max-adc_min)*1.0/2.0)+adc_min;
  Serial.printf("calibration n=%d Vmax=%d Vmin=%d Vmed=%d dt=%.2f\n",n,adc_max,adc_min,adc_zero,(1.0*sum_dt)/float(n));
  uint32_t tx1=0;
  uint32_t tx2=0;
  uint adc_v1=0;
  uint adc_v2=0;
  uint32_t adc_t1=0;
  uint32_t adc_t2=0;
  float m=0.0;

  double ffmin=3000.0;
  double ffmax=0.0;
  double ffsum=0.0;
  int k=0;
while (true) {
while (k<10) {
   
  adc_v1=adc_read();
  adc_t1=time_us_32();
  while (true) {

      adc_v2=adc_read();
      adc_t2=time_us_32();

      if (adc_v1 >= adc_zero) {
         if (adc_v2<adc_zero) {
            m=((adc_v2-adc_v1)*1.0/(adc_t2-adc_t1)*1.0);
            tx1=adc_t1+uint32_t((adc_zero-adc_v1)*1.0/m);        
            break;       
         }
      }
      adc_v1=adc_v2;
      adc_t1=adc_t2;
  }

  //*----- First epoch [tx1]
  
  adc_v1=adc_read();
  adc_t1=time_us_32();
  while (true) {

      adc_v2=adc_read();
      adc_t2=time_us_32();

      if (adc_v1>=adc_zero) {
          if (adc_v2<adc_zero) {
             m=((adc_v2-adc_v1)*1.0/(adc_t2-adc_t1)*1.0);
             tx2=adc_t1+uint32_t((adc_zero-adc_v1)*1.0/m);        
             break;       
          }
      }
      
      adc_v1=adc_v2;
      adc_t1=adc_t2;
  }

  //*------ Second epoch (tx2)
  
  double dfx=(tx2-tx1)*1.0;
  double T=dfx/1000.0;
  double f=1000.0/T;

  if (f>=1000.0) {
     Serial.printf("************Anomalous reading t2=%ld %t1=%ld v1=%ld v2=%ld zero=%ld tx2=%ld tx1=%ld dfx=%.2f T=%.2f f=%.2f\n",adc_t2,adc_t1,adc_v1,adc_v2,adc_zero,tx2,tx1,dfx,T,f);   
  }

  ffsum=ffsum+f;
  k++;
}

  double ffmed=ffsum/10.0;

  if (ffmed>=200.0 && ffmed <=3000.0) {
     if (ffmed<ffmin) ffmin=ffmed;
     if (ffmed>ffmax) ffmax=ffmed;
     Serial.printf("Freq f=%.2f min=%.2f max=%.2f k=%d sum=%.2f\n",ffmed,ffmin,ffmax,k,ffsum);   
  }
  ffsum=0.0;
  k=0;
  sleep_ms(100);
}
  
}
void loop() {
  // put your main code here, to run repeatedly:

}
void loop1() {}
