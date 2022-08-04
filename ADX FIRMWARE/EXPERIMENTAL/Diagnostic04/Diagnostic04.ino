
/*------
 * Diagnostic04
 * Program to test EEPROM in flash for the rp2040
 * PDX Project
 * 
 */

#include <EEPROM.h>

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

   #define BUILD              100
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*       DEFINITIONS RELATED TO THE USAGE OF EEPROM AS A PERMANENT STORAGE                     *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
   #define EEPROM_CAL          10
   #define EEPROM_BUILD        20
   #define EEPROM_TEMP         30
   #define EEPROM_MODE         40
   #define EEPROM_BAND         50
   uint32_t eeprom_tout=500;
   uint32_t tout=0;

   //#define EEPROM_CLR     1   //Initialize EEPROM (only to be used to initialize contents)
   #define EEPROM_SAVED 100     //Signature of EEPROM being updated at least once
   #define EEPROM_TOUT  500     //Timeout in mSecs to wait till commit to EEPROM any change


#define REPEAT_KEY   30
#define getGPIO(x) gpio_get(x)
#define setGPIO(x,y) gpio_put(x,y)
#define BOOL2CHAR(x)  (x==true ? "True" : "False")

bool     EEPROM_changed=false;

char     hi[80];
Si5351   si5351;       
uint32_t frequency =  7074000 * 100UL; //Fixed frequency
uint8_t  key=2;
uint8_t  mode=1;
uint16_t Band_slot=150000;
uint32_t cal_factor=12000000;

int led = LED_BUILTIN; // the PWM pin the LED is attached to
int brightness = 0;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED

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
/*------------------------------------------------------------------------------*
 * resetEEPROM                                                                  *
 * reset to pinche defaults                                                     *
 *------------------------------------------------------------------------------*/
void resetEEPROM() {

uint16_t save=EEPROM_SAVED;
uint16_t build=BUILD;

   mode=0;
   Band_slot=0;
   Serial.println("EEPROM Reset completed");
   updateEEPROM();
 
}
/*------
 * checkEEPROM
 * check if there is a pending EEPROM save that needs to be committed
 */
void checkEEPROM() {
    
    if((millis()-tout)>eeprom_tout && EEPROM_changed==true ) {
       updateEEPROM();
       EEPROM_changed=false;
       EEPROM.commit();
       Serial.println("EEPROM Saved!");       
    }
  
}
/*------------------------------------------------------------------------------*
 * updateEEPROM                                                                 *
 * selectively sets values into EEPROM                                          *
 *------------------------------------------------------------------------------*/
void updateEEPROM() {

uint16_t save=EEPROM_SAVED;
uint16_t build=BUILD;

   cal_factor++;
   mode++;
   Band_slot++;
   
   EEPROM.put(EEPROM_TEMP,save);
   EEPROM.put(EEPROM_BUILD,build);
   EEPROM.put(EEPROM_CAL,cal_factor);
   EEPROM.put(EEPROM_MODE,mode);
   EEPROM.put(EEPROM_BAND,Band_slot);

   EEPROM_changed=true;

   Serial.println("EEPROM updated and change stagged!");

}
/*----------------------------------------------------------*
 * Initialization function from EEPROM                      *
 *----------------------------------------------------------*/
void initEEPROM(){


 uint16_t temp=0;
 uint16_t save=EEPROM_SAVED;
 uint16_t build=0;

 
 EEPROM.get(EEPROM_TEMP,temp);
 
 
 if (temp != save){

    resetEEPROM();
 
    
 } else {

   /*-----------------------------------------------*
    * get configuration initialization from EEPROM  *            *
    ------------------------------------------------*/
   
  EEPROM.get(EEPROM_BUILD,build);
  EEPROM.get(EEPROM_CAL,cal_factor);

/*---- Kludge Fix   
 *     to overcome wrong initial values, should not difficult calibration
 */
   
  EEPROM.get(EEPROM_MODE,mode);
  EEPROM.get(EEPROM_BAND,Band_slot);

  sprintf(hi,"EEPROM read mode=%d Band_slot=%d cal_factor=%ld build=%d\n",mode,Band_slot,cal_factor,build);
  Serial. print(hi);

  updateEEPROM();
 }  
}  

// start reading from the first byte (address 0) of the EEPROM
int address = 0;
byte value;

void setup() {
  // initialize serial and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {}
  delay(2);
  Serial.flush();
   
  pinMode(led, OUTPUT);
  Serial.setTimeout(50);
  
  Serial.println("Diagnostic04 PDX Project EEPROM test");
  
  EEPROM.begin(512);
  initEEPROM();
}

void loop() {
 
  checkEEPROM();
   // set the brightness
  analogWrite(led, brightness);

  // change the brightness for next time through the loop:
  brightness = brightness + fadeAmount;

  // reverse the direction of the fading at the ends of the fade:
  if (brightness <= 0 || brightness >= 255) {
    fadeAmount = -fadeAmount;
  }

  // wait for 30 milliseconds to see the dimming effect
  delay(30);
  delay(500);
}
