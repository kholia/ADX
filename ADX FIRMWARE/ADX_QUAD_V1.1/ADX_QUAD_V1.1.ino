//*********************************************************************************************************
//********************* ADX - ARDUINO based DIGITAL MODES 4 BAND HF TRANSCEIVER ***************************
//********************************* Write up start: 02/01/2022 ********************************************
// FW VERSION: ADX_QUAD_V1.1 - Version release date: 04/11/2022
// Barb(Barbaros ASUROGLU) - WB2CBA - 2022
//*********************************************************************************************************
// Required Libraries
// ----------------------------------------------------------------------------------------------------------------------
// Etherkit Si5351 (Needs to be installed via Library Manager to arduino ide) - 
//          SI5351 Library by Jason Mildrum (NT7S) - https://github.com/etherkit/Si5351Arduino
// Arduino "Wire.h" I2C library(built-into arduino ide)
// Arduino "EEPROM.h" EEPROM Library(built-into arduino ide)
//*************************************[ LICENCE and CREDITS ]*********************************************
//  FSK TX Signal Generation code by: Burkhard Kainka(DK7JD) - http://elektronik-labor.de/HF/SDRtxFSK2.html
//  SI5351 Library by Jason Mildrum (NT7S) - https://github.com/etherkit/Si5351Arduino
//*-----------------------------------------------------------------------------------------------------------------*
//* Modified by Dr. P.E.Colla (LU7DZ)                                                                               
//*     - re-style of the code to facilitate customization for multiple boards
//*     - implement multiboard support (#define USDX 1)
//*         - remap of pushbuttons
//*         - (optional) rotary enconder
//*         - (optional) LCD display (same as uSDX)
//*     - changes to compatibilize with Pixino board (http://www.github.com/lu7did/Pixino
//*     - add CW support (includes keyer support)
//*     - add CAT support
//*     - add timeout & watchdog support
//* Forked version of the original ADX firmware located at http://www.github.com/lu7did/ADX
//*-----------------------------------------------------------------------------------------------------------------*
// License
// -------
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject
// to the following conditions:
// 
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
// ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
// CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

//*****************[ SI5351 VFO CALIBRATION PROCEDURE ]****************************************
// For SI5351 VFO Calibration Procedure follow these steps:
// 1 - Connect CAL test point and GND test point on ADX PCB to a Frequency meter or Scope that can measure 1 Mhz up to 1Hz accurately.
// 2 - Press SW2 / --->(CAL) pushbutton and hold.
// 4-  Power up with 12V or with 5V by using arduino USB socket while still pressing SW2 / --->(CAL) pushbutton. 
// 5 - FT8 and WSPR LEDs will flash 3 times and stay lit. Now Release SW2 / --->(CAL). Now Calibration mode is active.
// 6 - Using SW1(<---) and SW2(--->) pushbuttons change the Calibration frequency.
// 7 - Try to read 1 Mhz = 1000000 Hz exact on  Frequency counter or Oscilloscope. 
//     The waveform is Square wave so freqency calculation can be performed esaily.
// 7 - If you read as accurate as possible 1000000 Hz then calibration is done. 
// 8 - Power off ADX.
//*******************************[ LIBRARIES ]*************************************************
#include <si5351.h>
#include "Wire.h"
#include <EEPROM.h>
#include <stdint.h>
//********************************[ DEFINES ]***************************************************

#define USDX        1     //Use a modified uSDX board as base (D6/D7 --> D5/D8)
#define LEDS        1     //Use on-board LEDS
#define EE          1     //User EEPROM for persistence
#define PUSH        1     //Use UP-DOWN-TXSW Push buttons

/*----------------------------*
 * Pin Assignment             *
 *----------------------------*/
   #define AIN0        6           //(PD6)
   #define AIN1        7           //(PD7)
   #define TX         13           //(PB5) TX LED

#ifdef PUSH
   #define UP          2           //UP Switch
   #define DOWN        3           //DOWN Switch
   #define TXSW        4           //TX Switch
#endif //PUSH

#ifndef USDX
   #define RX          8           //RX Switch
#endif //USDX  -- Pin Assignment remap --

#ifdef LEDS
   #define WSPR        9           //WSPR LED 
   #define JS8        10           //JS8 LED
   #define FT4        11           //FT4 LED
   #define FT8        12           //FT8 LED
#endif //LEDS


/*------------------------------------*
 * General purpose global define      *
 * -----------------------------------*/
#define SI5351_REF  25000000UL  //change this to the frequency of the crystal on your si5351’s PCB, usually 25 or 27 MHz
#define CPU_CLOCK   16000000UL  //Processor clock

#define VOX_MAXTRY  10          //Max number of attempts to detect an audio incoming signal
#define CNT_MAX     65000       //Max count of timer1
#define FRQ_MAX     30000       //Max divisor for frequency allowed

#define TX_CLOCK    SI5351_CLK0 //TX Clock
#define RX_CLOCK    SI5351_CLK1 //RX Clock
#define CAL_CLOCK   SI5351_CLK2 //Callibration clock

#define BDLY        250
#define DELAY_WAIT  BDLY*4
#define DELAY_CAL   DELAY_WAIT/10

#ifdef EE
#define EEPROM_CAL  10
#define EEPROM_MODE 40
#define EEPROM_BAND 50
#define EEPROM_TEMP 30
#define EEPROM_SAVE 100
#endif //EEPROM

//*******************************[ VARIABLE DECLARATIONS ]*************************************

uint16_t mode;
uint16_t TX_State = 0;
uint16_t Band_slot;
uint16_t Band = 0;

unsigned long freq; 
unsigned long Cal_freq = 1000000UL; // Calibration Frequency: 1 Mhz = 1000000 Hz

unsigned long f[4]      = { 7074000, 7047500, 7078000, 7038600};   //Default frequency assignment   
unsigned long slot[6][4]={{ 3573000, 3575000, 3578000, 3568600},   //80m [0]
                          { 7074000, 7047500, 7078000, 7038600},   //40m [1]
                          {10136000,10140000,10130000,10138700},   //30m [2]
                          {14074000,14080000,14078000,14095600},   //20m [3]
                          {18100000,18104000,18104000,18104600},   //17m [4]
                          {21074000,21140000,21078000,21094600}};  //21m [5]                           

#ifdef LEDS
uint8_t  LED[4] ={FT8,FT4,JS8,WSPR};
#endif //LEDS  -- Board LEDS
 

//**********************************[ BAND SELECT ]************************************************

/*
 ADX can support up to 4 bands on board. Those 4 bands needs to be assigned to Band1 ... Band4 from supported 6 bands.
 To change bands press SW1 and SW2 simultaneously. Band LED will flash 3 times briefly and stay lit for the stored band. also TX LED will be lit to indicate
 that Band select mode is active. Now change band bank by pressing SW1(<---) or SW2(--->). When desired band bank is selected press TX button briefly to exit band select mode. 
 Now the new selected band bank will flash 3 times and then stored mode LED will be lit. 
 TX won't activate when changing bands so don't worry on pressing TX button when changing bands in band mode.


 Assign your prefered bands to B1,B2,B3 and B4
 Supported Bands are: 80m, 40m, 30m, 20m,17m, 15m
*/

uint16_t Bands[4]={40,30,20,17}; //Band1,Band2,Band3,Band4
/*--------------------------------------------------------------------------------------------*
 * Initialize DDS SI5351 object
 *--------------------------------------------------------------------------------------------*/
Si5351 si5351;

void setup_si5351() {
//------------------------------- SET SI5351 VFO -----------------------------------  
// The crystal load value needs to match in order to have an accurate calibration

  uint32_t cal_factor=0;

#ifdef EE
  EEPROM.get(EEPROM_CAL,cal_factor);
#endif //EEPROM
  
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  si5351.drive_strength(TX_CLOCK, SI5351_DRIVE_8MA);// SET For Max Power
  si5351.drive_strength(RX_CLOCK, SI5351_DRIVE_2MA); // Set for reduced power for RX 
  
}

/*-----------------------------------------------------------------------------------*
 * LED management functions
 *-----------------------------------------------------------------------------------*/
void resetLED() {               //Turn-off all LEDs

#ifdef LEDS  
   digitalWrite(WSPR, LOW); 
   digitalWrite(JS8, LOW); 
   digitalWrite(FT4, LOW); 
   digitalWrite(FT8, LOW); 
#endif //LEDS
 
}

void setLED(uint8_t LEDpin) {      //Turn-on LED {pin}
   resetLED();
   
#ifdef LEDS 
   uint8_t pin=LED[LEDpin];  
   digitalWrite(pin,HIGH);
#endif //LEDS

}

void blinkLED(uint8_t LEDpin) {    //Blink 3 times LED {pin}

#ifdef LEDS   
   uint8_t n=3;
   uint8_t pin=LED[LEDpin];
   while (n>0) {
       digitalWrite(pin,HIGH);
       delay(BDLY);
       digitalWrite(pin,LOW);
       delay(BDLY);
       n--;
   }
#endif //LEDS   
}

void callibrateLED(){           //Set callibration mode

#ifdef LEDS
   digitalWrite(WSPR, HIGH); 
   digitalWrite(FT8, HIGH);
   delay(DELAY_CAL);        
#endif //LEDS  
}
/*----------------------------------------------------------*
 * Mode assign
 *----------------------------------------------------------*/
void Mode_assign(){

#ifdef EE 
   EEPROM.get(EEPROM_MODE,mode);
#endif //EEPROM
   
   int i=4-mode;
   freq=f[i];
   setLED(i);

}
/*----------------------------------------------------------*
 * Frequency assign (band dependant)
 *----------------------------------------------------------*/
void Freq_assign(){

    uint8_t b;
    
    switch(Band) {
      case 80 : b=0;break;
      case 40 : b=1;break;
      case 30 : b=2;break;
      case 20 : b=3;break;
      case 17 : b=4;break;
      case 15 : b=5;break;
      default : b=1;break; 
    }
    for (int i=0;i>3;i++) {
      f[i]=slot[b,i];
    }
          
}

/*----------------------------------------------------------*
 * Band assignment based on selected slot
 *----------------------------------------------------------*/

void Band_assign(){

 resetLED();

#ifdef EE
 EEPROM.get(EEPROM_BAND,Band_slot);
#endif //EEPROM

 Band=Bands[3-Band_slot];
 blinkLED(slot[4-Band_slot]);
 delay(DELAY_WAIT);
 Freq_assign();
 Mode_assign();

}
/*----------------------------------------------------------*
 * Manually turn TX while pressed
 *----------------------------------------------------------*/
void ManualTX(){

#ifndef USDX  
    digitalWrite(RX,LOW);
#endif //USDX
    
    si5351.output_enable(RX_CLOCK, 0);   //RX off
  
    while(getTXSW()==LOW) {
       digitalWrite(TX,1);
       si5351.set_freq(freq*100ULL, TX_CLOCK);
       si5351.output_enable(TX_CLOCK, 1);   //TX on
       TX_State = 1;   
    }
    
    digitalWrite(TX,0); 
    si5351.output_enable(TX_CLOCK, 0);   //TX off
    si5351.output_enable(RX_CLOCK, 1);   //RX on
    TX_State = 0;

}
/*----------------------------------------------------------*
 * get value for a digital pin and return after debouncing
 *----------------------------------------------------------*/
bool getSwitch(uint8_t pin) {
  
    bool state=digitalRead(pin);
    if (state==HIGH) {
       delay(100);
       state=digitalRead(pin);
       if (state==HIGH) {
          return HIGH;
       } else {
         return LOW;   
       }
    } else {
      return LOW;
    }
  
}
/*----------------------------------------------------------*
 * read UP switch
 *----------------------------------------------------------*/
bool getUPState() {

#ifdef PUSH
    return getSwitch(UP);
#endif //PUSH 
}
/*----------------------------------------------------------*
 * read DOWN Switch
 *----------------------------------------------------------*/
bool getDOWNState() {

#ifdef PUSH
    return getSwitch(DOWN);
#endif //PUSH
}
/*----------------------------------------------------------*
 * read TXSW switch
 *----------------------------------------------------------*/
bool getTXSW() {
#ifdef PUSH
    return getSwitch(TXSW);
#endif //PUSH

}

/*----------------------------------------------------------*
 * Select band to operate
 *----------------------------------------------------------*/
void Band_Select(){

   digitalWrite(TX,1);

#ifdef EE   
   EEPROM.get(EEPROM_BAND,Band_slot);
#endif //EEPROM
   
   resetLED();
   blinkLED(4-Band_slot);
   
   while (true) {
      setLED(4-Band_slot);
      if ((getUPState() == LOW)&&(getDOWNState() == HIGH)) {
          Band_slot--;
          if (Band_slot < 1){
             Band_slot = 4;
          }
      } 
   
      if ((getUPState() == HIGH)&&(getDOWNState() == LOW)) {
         Band_slot++;
         if (Band_slot > 4){
            Band_slot = 1;
         }
      }                                               

      if (getTXSW() == LOW) {
         digitalWrite(TX,0);

#ifdef EE
         EEPROM.put(EEPROM_BAND,Band_slot);
#endif //EEPROM
         
         Band_assign();
         return; 
      } 
}

}
/*----------------------------------------------------------*
 * Calibration function
 *----------------------------------------------------------*/
void Calibration(){

  resetLED();
  uint8_t  n=4;
  uint32_t cal_factor=0;
   
  while (n>0) {
     callibrateLED();
     n--;
  }

#ifdef EE  
  EEPROM.get(EEPROM_CAL,cal_factor);
#endif //EEPROM
  
  while (true) {


     if (getUPState() == LOW) {
        cal_factor = cal_factor - 100;

#ifdef EE
        EEPROM.put(EEPROM_CAL, cal_factor); 
#endif //EEPROM
        
        si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
    
  // Set Calibration CLK output
  
        si5351.set_freq(Cal_freq * 100, CAL_CLOCK);
        si5351.drive_strength(CAL_CLOCK, SI5351_DRIVE_2MA); // Set for lower power for calibration
        si5351.set_clock_pwr(CAL_CLOCK, 1); // Enable the clock for calibration
     } 
   

     if (getDOWNState() == LOW) {
        cal_factor = cal_factor + 100;

#ifdef EE
        EEPROM.put(EEPROM_CAL, cal_factor);    
#endif //EEPROM
        
        si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);

  // Set Calbration Clock output
           
        si5351.set_freq(Cal_freq * 100, CAL_CLOCK);
        si5351.drive_strength(CAL_CLOCK, SI5351_DRIVE_2MA); // Set for lower power for Calibration
        si5351.set_clock_pwr(CAL_CLOCK, 1); // Enable clock2 
     } 

  }
}
/*----------------------------------------------------------*
 * Initialization function from EEPROM
 *----------------------------------------------------------*/

//*********************************[ INITIALIZATION FUNCTION ]******************************************
void INIT(){

 uint16_t temp;
 uint32_t cal_factor;

 #ifdef EE
 
 EEPROM.get(EEPROM_TEMP,temp);

 if (temp != EEPROM_SAVE){

    EEPROM.put(EEPROM_CAL,100000);
    EEPROM.put(EEPROM_MODE,4);
    EEPROM.put(EEPROM_TEMP,EEPROM_SAVE);
    EEPROM.put(EEPROM_BAND,1);


 } else {

   /*-----------------------------------------------*
    * get configuration initialization from EEPROM  *            *
    ------------------------------------------------*/

   EEPROM.get(EEPROM_TEMP,temp);
   EEPROM.get(EEPROM_CAL,cal_factor);
   EEPROM.get(EEPROM_MODE,mode);
   EEPROM.get(EEPROM_BAND,Band_slot);

 }  

#endif // EEPROM

 Band_assign();
 Freq_assign();
 Mode_assign();

 si5351.set_clock_pwr(TX_CLOCK, 0); // Turn off Calibration Clock

}

/*--------------------------------------------------------------------------*
 * definePinOut
 * isolate pin definition on a board conditional procedure out of the main
 * setup() flow
 *--------------------------------------------------------------------------*/
void definePinOut() {

#ifdef PUSH

   pinMode(UP,   INPUT);
   pinMode(DOWN, INPUT);
   pinMode(TXSW, INPUT);
#endif //PUSH

#ifndef USDX   
   pinMode(RX,   OUTPUT);
#endif //USDX define board


#ifdef LEDX
   pinMode(WSPR, OUTPUT);
   pinMode(JS8,  OUTPUT);
   pinMode(FT4,  OUTPUT);
   pinMode(FT8,  OUTPUT);
#endif //LEDS

   pinMode(TX,   OUTPUT);
   pinMode(6,    INPUT);  //PD6=AN0 must be grounded
   pinMode(7,    INPUT);  //PD7=AN1=HiZ

}

//*************************************[ SETUP FUNCTION ]************************************** 
void setup()
{
   definePinOut();
   INIT();
   setup_si5351();
  
   if ( getDOWNState() == LOW ) {
      Calibration();
    }
 
/*--------------------------------------------------------
 * initialize the timer1 as an analog comparator
 *--------------------------------------------------------*/
  TCCR1A = 0x00;
  TCCR1B = 0x01; // Timer1 Timer 16 MHz
  TCCR1B = 0x81; // Timer1 Input Capture Noise Canceller
  ACSR |= (1<<ACIC);  // Analog Comparator Capture Input
  
  pinMode(7, INPUT); //PD7 = AN1 = HiZ, PD6 = AN0 = 0

#ifndef USDX  
  digitalWrite(RX,LOW);
#endif //USDX

  Mode_assign(); 

}

//*=*=*=*=*=*=*=*=*=*=*=*=*=[ END OF SETUP FUNCTION ]*=*=*=*=*=*=*=*=*=*=*=*=
/*---------------------------------------------------------------------------*
 *  checkMode
 *  manage change in mode during run-time
 *---------------------------------------------------------------------------*/
void checkMode() {


  if ((getUPState() == LOW)&&(getDOWNState() == LOW)&&(TX_State == 0)) {
     Band_Select();
  }

  if ((getUPState() == LOW)&&(getDOWNState() == HIGH)&&(TX_State == 0)) {
     mode = mode - 1;
     if (mode < 1){
        mode = 4;
     }

#ifdef EE
     EEPROM.put(EEPROM_MODE, mode); 
#endif //EEPROM
     
     Mode_assign();
  } 
   

  if ((getUPState() == HIGH) && (getDOWNState() == LOW)&&(TX_State == 0)) {
     mode = mode + 1;
     if (mode > 4){
        mode = 1;
     }

#ifdef EEPROM
     EEPROM.put(EEPROM_MODE, mode); 
#endif //EEPROM
     
     Mode_assign();
  } 


if ((getTXSW() == LOW) && (TX_State == 0)) {
   Mode_assign();
   ManualTX();
 }
 
}
//***************************[ Main LOOP Function ]**************************
void loop()
{  

  checkMode();

/*----------------------------------------------------------------------------------*
 * main transmission loop                                                           *
 * Timer1 (16 bits) with no pre-scaler (16 MHz) is checked to detect zero crossings *
 * if there is no overflow the frequency is calculated                              *
 * if activity is detected the TX is turned on                                      *
 * TX mode remains till no further activity is detected (operate like a VOX command)*
 *----------------------------------------------------------------------------------*/

uint16_t FSK   = VOX_MAXTRY;
uint16_t FSKtx = 0;

 while (FSK>0){                                 //Iterate up to 10 times looking for signal to transmit
    TCNT1 = 0;                                  //While this iteration is performed if the TX is off 
    while (ACSR &(1<<ACO)){                     //the receiver is operating with autonomy
       if (TCNT1>CNT_MAX) break;
    }
    while ((ACSR &(1<<ACO))==0){
       if (TCNT1>CNT_MAX) break;
    }
    TCNT1 = 0;
    while (ACSR &(1<<ACO)){
      if (TCNT1>CNT_MAX) break;
    }
    uint16_t d1 = ICR1;  
    while ((ACSR &(1<<ACO))==0){
      if (TCNT1>CNT_MAX) break;
    } 
    while (ACSR &(1<<ACO)){
      if (TCNT1>CNT_MAX) break;
    }
    uint16_t d2 = ICR1;

/*-----------------------------------------------------*
 * end of waveform measurement, now check what is the  *
 * input frequency                                     *
 *-----------------------------------------------------*/
    if (TCNT1 < CNT_MAX){
       if ((d2-d1) == 0) break;
       unsigned long codefreq = CPU_CLOCK/(d2-d1);
       if ((codefreq < FRQ_MAX) && (codefreq > 0)){
          if (FSKtx == 0){
             TX_State = 1;
             digitalWrite(TX,HIGH);

#ifndef USDX
             digitalWrite(RX,LOW);
#endif //USDX
             
             si5351.output_enable(RX_CLOCK, 0);   //RX off
             si5351.output_enable(TX_CLOCK, 1);   //TX on
          }
          si5351.set_freq((freq * 100 + codefreq), TX_CLOCK);  
          FSKtx = 1;
       }
    } else {
       FSK--;
    }
 }

/*---------------------------------------------------------------------------------*
 * when out of the loop no further TX activity is performed, therefore the TX is   *
 * turned off and the board is set into RX mode                                    *
 *---------------------------------------------------------------------------------*/
 digitalWrite(TX,0);                      //TX signal turned off
 si5351.output_enable(TX_CLOCK, 0);       //TX CLK off
 si5351.set_freq(freq*100ULL, RX_CLOCK);  //Change frequency of RX clock  
 si5351.output_enable(RX_CLOCK, 1);       //RX on
 TX_State = 0;  
 
#ifndef USDX 
 digitalWrite(RX,HIGH);                   //
#endif //USDX No RX line used 
 
 FSKtx = 0;                               //Prepare for next cycle
     
}
//*********************[ END OF MAIN LOOP FUNCTION ]*************************

//********************************[ END OF INITIALIZATION FUNCTION ]*************************************
