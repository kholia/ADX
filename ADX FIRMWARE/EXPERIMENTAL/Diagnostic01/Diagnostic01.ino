/*------
 * Diagnostic01
 * Program to test LED and Switches connection on the ADX board
 * PDX Project
 * 
 */
#include "pico/stdlib.h"
#include "pico/binary_info.h"

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

uint8_t led=2;
char hi[80];

/*-----------------------------------------------------------------------------*
 * setLED                                                                      *
 * set a given LED                                                             *
 *-----------------------------------------------------------------------------*/
void setLED(uint8_t pin) {


   gpio_put(TX,  LOW);
   gpio_put(FT8, LOW);
   gpio_put(FT4, LOW);
   gpio_put(JS8, LOW);
   gpio_put(WSPR,LOW);

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

   Serial.begin(115200);
   Serial.println("PDX Diagnostic Program 01");
   Serial.println("Press any switch to lit LEDs");

  
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
    if (detectKey(TXSW,LOW,true) == LOW) { led++; f=true;}
    if (detectKey(UPSW,LOW,true) == LOW) { led++; f=true;}
    if (detectKey(DNSW,LOW,true) == LOW) { led++; f=true;}

    if (led > 7) {
      led = 2;
    }

/*----    
 * if any key pressed just lit a LED
 */
    if (f==true) {
       setLED(led);
    }   
}
