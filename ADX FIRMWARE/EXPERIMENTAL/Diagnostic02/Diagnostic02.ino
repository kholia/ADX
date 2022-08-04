
/*------
 * Diagnostic02
 * Program to test Si5351 Clock with LED and Switches connection on the ADX board
 * PDX Project
 * 
 */
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include <Wire.h>
#include <si5351.h>

#define WSPR 7
#define JS8  6
#define FT4  5
#define FT8  4
#define TX   3

#define TXSW 9
#define UPSW 10
#define DNSW 11


#define REPEAT_KEY   30
#define getGPIO(x) gpio_get(x)
#define setGPIO(x,y) gpio_put(x,y)
#define BOOL2CHAR(x)  (x==true ? "True" : "False")

uint8_t  led=2;
char     hi[80];
Si5351   si5351;       
uint32_t frequency =  7074000 * 100UL; //Fixed frequency
uint8_t  key=2;

/*-----------------------------------------------------------------------------*
 * resetLED                                                                    *
 * reset all LEDS                                                              *
 *-----------------------------------------------------------------------------*/
void resetLED() {
   gpio_put(TX,  LOW);
   gpio_put(FT8, LOW);
   gpio_put(FT4, LOW);
   gpio_put(JS8, LOW);
   gpio_put(WSPR,LOW);
   
}

void setLED(uint8_t pin) {
   gpio_put(pin,HIGH);

   sprintf(hi,"%s set LED(%d)\n",__func__,pin);
   Serial.print(hi);

   

}
/*-----------------------------------------------------------------------------*
 * detectKey                                                                   *
 * detect if a push button is pressed                                          *
 *-----------------------------------------------------------------------------*/
bool detectKey(uint8_t k, bool v, bool w) {

   uint32_t tdown=millis();
   if (getGPIO(k)==v) {    
      while (millis()-tdown<REPEAT_KEY) {
      }
      if (getGPIO(k)==v) { //confirmed as v value now wait for the inverse, if not return the inverse      
          
          if (w==false) {
             sprintf(hi,"%s %s\n",__func__,BOOL2CHAR(v));
             Serial.print(hi);
             return v;
          }
          
          while (true) {

              if (getGPIO(k)!=v) {
                 tdown=millis();
                 while (millis()-tdown<REPEAT_KEY) {
                 }
                 if (getGPIO(k)!=v) {
                    sprintf(hi,"%s %s\n",__func__,BOOL2CHAR(v));
                    Serial.print(hi);
                    return v;
                 }
              }
          }
          sprintf(hi,"%s %s\n",__func__,BOOL2CHAR(v));
          Serial.print(hi);
          return !v;
                     
          }
   }   
   return !v;
}

/*------
 * setup
 */
void setup() {


   gpio_init(TX);
   gpio_init(UPSW);
   gpio_init(DNSW);
   gpio_init(TXSW);
   gpio_init(WSPR);
   gpio_init(JS8);
   gpio_init(FT4);
   gpio_init(FT8);

   gpio_set_dir(UPSW,GPIO_IN);
   gpio_set_dir(DNSW,GPIO_IN);
   gpio_set_dir(TXSW,GPIO_IN);

   gpio_set_dir(TX, GPIO_OUT);
   gpio_set_dir(WSPR,GPIO_OUT);
   gpio_set_dir(JS8,GPIO_OUT);
   gpio_set_dir(FT4,GPIO_OUT);
   gpio_set_dir(FT8,GPIO_OUT);

   Wire.setSDA(16);
   Wire.setSCL(17);
   Wire.begin();

   Serial.begin(115200);
   Serial.println("PDX Diagnostic Program 01");
   Serial.println("Press any switch to lit LEDs and CLK");


  // Initialize the Si5351
   int rc = si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
   if (rc != true) {
      Serial.flush();
      sprintf(hi,"error while initializing clock generator Si5351 rc(%d)\n",rc);
      Serial.print(hi);
   }
   si5351.set_clock_pwr(SI5351_CLK0, 0); // safety first
   si5351.set_clock_pwr(SI5351_CLK1, 0); // safety first
   si5351.set_clock_pwr(SI5351_CLK2, 0); // safety first


   si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA); // Set for maximum power
   si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_8MA); // Set for maximum power
   si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_8MA); // Set for maximum power

   si5351.set_clock_pwr(SI5351_CLK0, 0); // Disable the clock initially
   si5351.set_clock_pwr(SI5351_CLK1, 0); // Disable the clock initially
   si5351.set_clock_pwr(SI5351_CLK2, 0); // Disable the clock initially 
 
}

/*-----
 * loop
 */

void loop() {

  
    bool f=false;

/*----     
 * if any switch is pressed then lit the following LED round-robin
 * lit a fake LED (2) to turn off them all
 */

    if (detectKey(TXSW,LOW,true) == LOW) { 
       key++;
       f=true;
       sprintf(hi,"key(%d)\n",key);
       Serial.print(hi);
    }  

    if (detectKey(UPSW,LOW,true) == LOW) { 
       key++;
       f=true;
       sprintf(hi,"key(%d)\n",key);
       Serial.print(hi);

    }  

    if (detectKey(DNSW,LOW,true) == LOW) { 
       key++;
       f=true;
       sprintf(hi,"key(%d)\n",key);
       Serial.print(hi);
    }  

    if (key>7) {
       key=2;
       sprintf(hi,"reset key(%d)\n",key);
       Serial.print(hi);

    }
 
    if (f==true && key==3) { 
      resetLED();
      setLED(WSPR);
      setLED(TX);
      si5351.set_clock_pwr(SI5351_CLK0, 1);
      si5351.output_enable(SI5351_CLK1, 0);
      si5351.output_enable(SI5351_CLK1, 0);
      si5351.output_enable(SI5351_CLK2, 0);
      si5351.set_freq(frequency, SI5351_CLK0);
      si5351.output_enable(SI5351_CLK0, 1);

      sprintf(hi,"CLK0 enabled freq=%ld",frequency);    
      Serial.println(hi);
    }
    
    if (f==true && key==4) {
      resetLED();
      setLED(JS8);
      setLED(TX);
      si5351.set_clock_pwr(SI5351_CLK1, 1);
      si5351.output_enable(SI5351_CLK0, 0);
      si5351.output_enable(SI5351_CLK1, 0);
      si5351.output_enable(SI5351_CLK2, 0);
      si5351.set_freq(frequency, SI5351_CLK1);    
      si5351.output_enable(SI5351_CLK1, 1);

      sprintf(hi,"CLK1 enabled freq=%ld",frequency);    
      Serial.println(hi);

    }
    if (f==true && key==5) { 
       resetLED();
       setLED(FT4);
       setLED(TX);
       si5351.set_clock_pwr(SI5351_CLK2, 1);
       si5351.output_enable(SI5351_CLK0, 0);
       si5351.output_enable(SI5351_CLK1, 0);
       si5351.output_enable(SI5351_CLK2, 0);
       si5351.set_freq(frequency, SI5351_CLK2);    
       si5351.output_enable(SI5351_CLK2, 1);
       sprintf(hi,"CLK2 enabled freq=%ld",frequency);    
       Serial.println(hi);

    }

    if (f==true && (key==6 || key==7)) {
       resetLED();
       setLED(FT8);
       si5351.set_clock_pwr(SI5351_CLK0, 0);
       si5351.output_enable(SI5351_CLK1, 0);
       si5351.output_enable(SI5351_CLK2, 0);
    }
    if (led > 7) {
      led = 2;
    }

}
