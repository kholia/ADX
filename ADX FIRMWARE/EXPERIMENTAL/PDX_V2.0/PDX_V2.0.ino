//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=**=*=*
//*                           PDX - PICO based DIGITAL MODES 4 BAND HF TRANSCEIVER                           *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=**=*=*
// Firmware Version 2.0
//
// Barb (Barbaros Asuroglu) - WB2CBA - 2022
// Dhiru (Dhiru Kholia)     - VU3CER - 2022
// Pedro (Pedro Colla)      - LU7DZ  - 2022
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=**=*=*
// Required Libraries
// Etherkit Si5351 (Needs to be installed via Library Manager to arduino ide) -
//          SI5351 Library by Jason Mildrum (NT7S) - https://github.com/etherkit/Si5351Arduino
// Arduino "Wire.h" I2C library(built-into arduino ide)
// Arduino "EEPROM.h" EEPROM Library(built-into arduino ide)
// AVR "wdt.h" Watchdog Library
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=**=*=*
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
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=**=*=*

#include <Arduino.h>
#include <stdint.h>
#include "pdx_common.h"

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                         Compilation conditional messages                                    *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
#pragma message( "Compiling " __FILE__ )
#pragma message( "Last modified on " __TIMESTAMP__ )

#ifdef DEBUG
#pragma message ("* Debug messages enabled  *")
#ifdef DEBUG_UART
#pragma message ("* Debug messages output thru the UART1 (serial) output  *")
#else
#pragma message ("* Debug messages output thru the USB Serial output      *")
#endif
#endif //DEBUG 

/*-----------------------------------------------------------------------------------------------*
 * In order to establish the initial WiFi credentials at compilation time you might create it
 * editing a file named wifi_credentials.h with the following content
 * 
 * #define WIFI_SSID     "your-ssid"         //(i.e  #define WIFI_SSID "Fibertel WiFi996 2.4GHz")
 * #define WIFI_PASW     "your-password"     //(i.e. #define WIFI_PASW "1234567890"

 * Place this file in the same directory the rest of the firmware is in order to be incorporated
 * as part of the compilation process if the TCP/IP functionality is enabled (#define WIFI 1)
 *------------------------------------------------------------------------------------------------*/
#ifdef WIFI
#if __has_include("wifi_credentials.h")
#pragma message("Wifi credentials found")
#else
#pragma message("Wifi credentials not found, please supply to enable TCP/IP related functionality")
#undef WIFI
#endif
#endif //WIFI
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                            Data definitions                                                 *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*

uint32_t ffsk     = 0;
int      pwm_slice;
uint32_t f_hi;
uint32_t fclk     = 0;
int32_t  error    = 0;
uint32_t codefreq = 0;
uint32_t prevfreq = 0;

/*------------------------------*
   Epoch values
  ------------------------------*/
uint32_t t1[2];
uint32_t t2[2];
uint32_t v1[2];
uint32_t v2[2];

#ifdef ADCZ
/*-------------------------------*
   Sample data
*/
uint16_t adc_v1 = 0;
uint16_t adc_v2 = 0;
uint32_t adc_t1 = 0;
uint32_t adc_t2 = 0;

bool     adc_high = false;
bool     adc_low = false;

/*-------------------------------*
   Computed frequency limits
*/
double   ffmin = FSKMAX;
double   ffmax = FSKMIN;
uint16_t adc_min = ADCMAX;
uint16_t adc_max = ADCMIN;
uint16_t adc_zero = ADCZERO;
uint16_t adc_uh;
uint16_t adc_ul;
/*--------------------------------*
   FSM state variable
  --------------------------------*/
uint8_t  QSTATE = 0;
#endif //ADCZ



//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*       GLOBAL VARIABLE DEFINITION                                                            *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
char     hi[80];
uint16_t bounce_time    = BOUNCE_TIME;
uint16_t short_time     = SHORT_TIME;
uint16_t vox_maxtry     = VOX_MAXTRY;
int      cnt_max        = CNT_MAX;
uint16_t max_blink      = MAX_BLINK;
uint8_t  SSW            = 0;          //System SSW variable (to be used with getWord/setWord)
uint8_t  TSW            = 0;          //System timer variable (to be used with getWord/setWord);
uint8_t  QSW            = 0;
uint16_t mode           = 0;          //Default to mode=0 (FT8)
uint16_t Band_slot      = 0;          //Default to Bands[0]=40
int32_t  cal_factor     = 0;
unsigned long Cal_freq  = 1000000UL; // Calibration Frequency: 1 Mhz = 1000000 Hz
unsigned long f[MAXMODE]                   =  { 7074000, 7047500, 7078000, 7038600, 7030000};       //Default frequency assignment
const unsigned long slot[MAXBAND][MAXMODE] = {{ 3573000, 3575000, 3578000, 3568600, 3560000},       //80m [0]
  { 5357000, 5357000, 5357000, 5287200, 5346500},       //60m [1]
  { 7074000, 7047500, 7078000, 7038600, 7030000},       //40m [2]
  {10136000, 10140000, 10130000, 10138700, 10106000},   //30m [3]
  {14074000, 14080000, 14078000, 14095600, 14060000},   //20m [4]
  {18100000, 18104000, 18104000, 18104600, 18096000},   //17m [5]
  {21074000, 21140000, 21078000, 21094600, 21060000},   //15m [6]
  {24915000, 24915000, 24922000, 24924600, 24906000},   //12m [7]
  {28074000, 28074000, 28078000, 28124600, 28060000}
};  //10m [8]

unsigned long freq      = f[mode];
const uint8_t LED[4]    = {FT8, FT4, JS8, WSPR}; //A 5th virtual mode is handled if CW enabled, LEDS are managed in that case not using this table

/*-------------------------------------*
   Manage button state
  -------------------------------------*/
uint8_t       button[3]   = {0, 0, 0};
unsigned long downTimer[3] = {PUSHSTATE, PUSHSTATE, PUSHSTATE};


/*-------------------------------------*
   Control communication across cores
  -------------------------------------*/

double fsequences[NFS]; // Ring buffer for communication across cores
int nfsi   = 0;
double pfo = 0; // Previous output frequency


//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*       GLOBAL VARIABLE DEFINITION CONDITIIONAL TO DIFFERENT OPTIONAL FUNCTIONS               *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*


#ifdef WDT
uint32_t      wdt_tout    = 0;
#endif //WDT

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                    CODE INFRASTRUCTURE                                                      *
//* General purpose procedures and functions needed to implement services through the code      *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
/*-------------------------------------------*
   delay_uSec
   A fast delay function
  -------------------------------------------*/
void delay_uSec(uint16_t d) {

  uint32_t t = time_us_32() + d;
  while (t > time_us_32());

}
/*-------------------------------------------*
   getWord
   get boolean bitwise pseudo-variable
  -------------------------------------------*/
bool getWord (uint8_t SysWord, uint8_t v) {
  return SysWord & v;
}

/*-------------------------------------------*
   setSSW
   set boolean bitwise pseudo-variable
  -------------------------------------------*/
void setWord(uint8_t* SysWord, uint8_t v, bool val) {
  *SysWord = ~v & *SysWord;
  if (val == true) {
    *SysWord = *SysWord | v;
  }
}


#ifdef CAT

// Forward reference prototypes
void switch_RXTX(bool t);     //advanced definition for compilation purposes (interface only)
void Mode_assign();           //advanced definition for compilation purposes
//void Freq_assign();
void Band_assign(bool l);
uint16_t changeBand(uint16_t c);

#endif //CAT

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                   Si5351 MANAGEMENT SUBSYSTEM                                               *
//* Most of the work is actually performed by the Si5351 Library used                           *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
Si5351 si5351;

/*--------------------------------------------------------------------------------------------*
   Initialize DDS Si5351 object
  --------------------------------------------------------------------------------------------*/
void setup_si5351() {
  //------------------------------- SET Si5351 VFO -----------------------------------
  // The crystal load value needs to match in order to have an accurate calibration
  //---------------------------------------------------------------------------------
  long cal = XT_CAL_F;

  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);// SET For Max Power
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA);// Set for reduced power for RX

#ifdef DEBUG
  _INFO;
#endif // DEBUG
}

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                   LED MANAGEMENT SUBSYSTEM                                                  *
//* Functions to operate the 5 LED the ADX board has                                            *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
/**********************************************************************************************/
/*                                      LED Management                                        */
/**********************************************************************************************/
void clearLED(uint8_t LEDpin) {
  setGPIO(LEDpin, LOW);

#ifdef DEBUG
  _EXCPLIST("%s pin=%d\n", __func__, LEDpin);
#endif //DEBUG
}
/*-----
   Turn off all LEDs
*/
void resetLED() {  // Turn-off all LEDs
  clearLED(WSPR);
  clearLED(JS8);
  clearLED(FT4);
  clearLED(FT8);

#ifdef DEBUG
  _EXCP;
#endif //DEBUG
}

/*-----
   Set a particular LED ON
*/
void rstLED(uint8_t LEDpin, bool clrLED) {     //Turn-on LED {pin}

  (clrLED == true ? resetLED() : void(_NOP));
  setGPIO(LEDpin, LOW);

#ifdef DEBUG
  _EXCPLIST("%s(%d)\n", __func__, LEDpin);
#endif //DEBUG

}

/*-----
   Set a particular LED ON
*/
void setLED(uint8_t LEDpin, bool clrLED) {     //Turn-on LED {pin}

  if (clrLED == true) {
    resetLED();
  }

  //(clrLED == true ? resetLED() : void(_NOP));
  setGPIO(LEDpin, HIGH);


#ifdef DEBUG
  _EXCPLIST("%s(%d)\n", __func__, LEDpin);
#endif //DEBUG
}

/*-------
   Blink a given LED
*/
void blinkLED(uint8_t LEDpin) {    //Blink 3 times LED {pin}

#ifdef DEBUG
  _EXCPLIST("%s (%d)\n", __func__, LEDpin);
#endif //DEBUG

  uint8_t n = (max_blink - 1);

  while (n > 0) {
    setGPIO(LEDpin, HIGH);
    delay(BDLY);
    setGPIO(LEDpin, LOW);
    delay(BDLY);
    n--;

#ifdef WDT
    wdt_reset();
#endif //WDT
  }
}

/*-------
 * Blink all LEDs
 */

void blinkAllLED() {
   uint8_t n=(max_blink-1);

   while (n>0) {
       for (int j=0;j<4;j++) {
           setGPIO(FT8+j,HIGH);
       }
       delay(BDLY);
       for (int j=0;j<4;j++) {
           setGPIO(FT8+j,LOW);
       }
       delay(BDLY);
       n--;

       #ifdef WDT       
          wdt_reset();
       #endif //WDT       
   } 
}


/*-----
   LED on calibration mode
*/
void calibrateLED() {          //Set callibration mode
  setGPIO(WSPR, HIGH);
  setGPIO(FT8, HIGH);
  delay(DELAY_CAL);
#ifdef DEBUG
  _INFO;
#endif //DEBUG
}

/*-----
   Signal band selection with LED   (THIS NEEDS TO BE REVIEWED TO ACTUALLY SHOW MORE THAN 4 BANDS
*/
void bandLED(uint16_t b) {         //b would be 0..3 for standard ADX or QUAD
  setLED(LED[3 - b], true);
}

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                   BUTTON MANAGEMENT SUBSYSTEM                                               *
//* Functions to operate the 3 push buttons the ADX board has                                   *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*

/*-----------------------------------------------------------------------------*
   detectKey
   detect if a push button is pressed
  -----------------------------------------------------------------------------*/
bool detectKey(uint8_t k, bool v, bool w) {

  uint32_t tdown = millis();
  if (getGPIO(k) == v) {
    while (millis() - tdown < REPEAT_KEY) {

#ifdef WDT
      wdt_reset();
#endif //WDT

    }
    if (getGPIO(k) == v) { //confirmed as v value now wait for the inverse, if not return the inverse
      if (w == false) {
        return v;
      }

#ifdef DEBUG
      _INFOLIST("%s <STUCK> about to wait for switch(%d) value(%s) to go up\n", __func__, k, BOOL2CHAR(v));
#endif //DEBUG

      while (true) {
#ifdef WDT
        wdt_reset();
#endif //WDT

        if (getGPIO(k) != v) {
          tdown = millis();
          while (millis() - tdown < REPEAT_KEY) {
#ifdef WDT
            wdt_reset();
#endif //WDT
          }
          if (getGPIO(k) != v) {
#ifdef DEBUG
            _INFOLIST("%s switch(%d) value(%s)\n", __func__, k, BOOL2CHAR(v));
#endif //DEBUG

            return v;
          }
        }
      }
      return !v;
    }
  }
  return !v;
}

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                   TRANSCEIVER MANAGEMENT SUBSYSTEM                                          *
//* Functions related to the overall operation of the transceiver                               *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*

/*---------------------------------------------------------------------------------------------*
   Switch between RX and TX
  ---------------------------------------------------------------------------------------------*/
void switch_RXTX(bool t) { // t = False (RX) : t = True (TX)
  if (t) { // Set to TX
    /*-----------------------------------*
       if Watchdog enabled avoid to turn
       TX on if the watchdog mark hasn't
       been cleared.
      -----------------------------------*/
#ifdef WDT
    if (getWord(TSW, TX_WDT) == HIGH) {
      return;
    }
#endif //WDT
    /*-----------------------------------*
                     TX
      -----------------------------------*/
    setGPIO(RX, LOW);
    si5351.output_enable(SI5351_CLK1, 0);   //RX off
    uint32_t freqtx = freq;
#ifdef CW
    if (mode == MAXMODE - 1) {
      freqtx = freq + uint32_t(cwshift);
    } else {
      freqtx = freq;
    }
#ifdef DEBUG
    _INFOLIST("%s TX+ (CW=%s) TX=%s ftx=%ld f=%ld\n", __func__, BOOL2CHAR(getWord(SSW, CWMODE)), BOOL2CHAR(getWord(SSW, TXON)), freqtx, freq);
#endif //DEBUG

#else   //!CW
    freqtx = freq;
#ifdef DEBUG
    _INFOLIST("%s TX+ f=%ld\n", __func__, freqtx);
#endif //DEBUG
#endif //CW

    si5351.set_freq(freqtx * 100ULL, SI5351_CLK0);
    si5351.output_enable(SI5351_CLK0, 1);   // TX on

    setGPIO(TX, HIGH);
    setWord(&SSW, TXON, HIGH);

#ifdef WDT
    wdt_tout = millis();
#endif //WDT

    return;
  }

  // RX
#ifdef CAT
  if (getWord(SSW, CATTX) == true) {
    return; // If the PTT is managed by the CAT subsystem get out of the way
  }
#endif // CAT

  setGPIO(RX, HIGH);
  si5351.output_enable(SI5351_CLK0, 0); // TX off

#ifdef DEBUG
  if (getWord(SSW, TXON) == HIGH) {
    _INFOLIST("%s RX+ f=%ld\n", __func__, freq);
  }
#endif //DEBUG

  // set to master frequency
  si5351.set_freq(freq * 100ULL, SI5351_CLK1);
  si5351.output_enable(SI5351_CLK1, 1); // RX on

  setGPIO(TX, 0);
  setWord(&SSW, TXON, LOW);
  setWord(&SSW, VOX, LOW);
}

/*----------------------------------------------------------*
   Manually turn TX while pressed
  ----------------------------------------------------------*/
bool getTXSW();  // prototype for forward reference
void ManualTX() {

  bool buttonTX = getTXSW();
  switch_RXTX(HIGH);

#ifdef DEBUG
  _INFOLIST("%s ManualTX(HIGH)\n", __func__);
#endif // DEBUG

  while (buttonTX == LOW) {

#ifdef WDT
    wdt_reset();

    if ((millis() > (wdt_tout + uint32_t(WDT_MAX))) && getWord(SSW, TXON) == HIGH) {
      switch_RXTX(LOW);
      setWord(&TSW, TX_WDT, HIGH);
      wdt_tout = millis();
      return;
    }

#ifdef CAT
    serialEvent();
#endif //CAT

#endif //WDT
    buttonTX = getTXSW();

  }
  switch_RXTX(LOW);
#ifdef DEBUG
  _INFOLIST("%s ManualTX(LOW)\n", __func__);
#endif //DEBUG
}


/*--------------------------------------------------------------*
   getTXSW() -- read TXSW switch
   This switch still required debouncing but might operate
   over long pulsing periods because of the manual TX function
   and CW operation. It doesn't require to distinguish between
   short and long pulse though.
  ---------------------------------------------------------------*/
bool getTXSW() {
  return detectKey(TXSW, LOW, false);
}

/*==================================================================================================*
   Clock (Si5351) Calibration methods
   Legacy method (ADX)
       Clock (CLK2) is set to 1MHz output , calibration factor is increased (UP) or decreased (DOWN)
       until a frequency counter shows 1 MHz, this way any offset on the clock will be compensated
       calibration factor will be stored in EEPROM and saved till next calibration
    Automatic method (PDX)
       Clock (CLK2) is set to 10MHz output, the board connects this value to the GPIO8 (CAL) pin.
       An iteration is made automatically until the read value is 10MHz.
       The calibration factor will be store
  ==================================================================================================*/


/**********************************************************************************************/
/*                               Operational state management                                 */
/**********************************************************************************************/
/*----------------------------------------------------------*
   Band increase
  ----------------------------------------------------------*/
uint16_t changeBand(uint16_t c) {
  uint16_t b = (Band_slot + c) % BANDS;
#ifdef DEBUG
  _INFOLIST("%s() change=%d Band_slot=%d b=%d\n", __func__, c, Band_slot, b);
#endif //DEBUG
  return b;
}

/*----------------------------------------------------------*
   Mode assign
  ----------------------------------------------------------*/
void Mode_assign() {


#ifdef CW
  if (mode == MAXMODE - 1) {
    resetLED();
    setLED(JS8, false);
    setLED(FT4, false);
  } else {
    setLED(LED[mode], true);
  }
#else
  setLED(LED[mode], true);
#ifdef DEBUG
  _INFOLIST("%s LED mode=%d LED=%d true\n", __func__, mode, LED[mode]);
#endif //DEBUG
#endif //CW
  /*---------------------------------------*
     Change the frequency different from what
     it is required by the mode and then
     change the DDS accordingly
    ---------------------------------------*/
  if (freq != f[mode]) {    //@@@
    freq = f[mode];        //@@@
    switch_RXTX(LOW);
  }                        //@@@
  setWord(&SSW, VOX, false);
  setWord(&SSW, TXON, false);
#ifdef WDT
  wdt_reset();
#endif

#ifdef DEBUG
  _INFOLIST("%s mode(%d) f(%ld)\n", __func__, mode, f[mode]);
#endif //DEBUG
}
/*----------------------------------------------------------*
   Select band to operate
  ----------------------------------------------------------*/
void Band_Select() {

  resetLED();

#ifdef DEBUG
  _INFOLIST("%s slot(%d) LED(%d)\n", __func__, Band_slot, LED[3 - Band_slot]);
#endif //DEBUG

  blinkLED(LED[3 - Band_slot]);
  setLED(LED[3 - Band_slot], true);
  bool     l = false;
  uint32_t t = millis();

  while (true) {
#ifdef WDT
    wdt_reset();
#endif //WDT

    if ((millis() - t) > uint32_t(BDLY)) {
      t = millis();
#ifdef DEBUG
      _EXCPLIST("%s blink TX led\n", __func__);
#endif //DEBUG
      (l == false ? setLED(TX, false) : clearLED(TX));
      l = !l;
    }

    if (detectKey(UP, LOW, NOWAIT) == LOW) {
#ifdef DEBUG
      _INFOLIST("%s Key UP detected\n", __func__);
#endif //DEBUG

      while (detectKey(UP, LOW, WAIT) == LOW) {}

#ifdef DEBUG
      _INFOLIST("%s Key UP released\n", __func__);
#endif //DEBUG

      Band_slot = changeBand(-1);
      setLED(LED[3 - Band_slot], true);

#ifdef DEBUG
      _INFOLIST("%s slot(%d)\n", __func__, Band_slot);
#endif //DEBUG
    }

    if (detectKey(DOWN, LOW, WAIT) == LOW) {

#ifdef DEBUG
      _INFOLIST("%s Key DOWN detected\n", __func__);
#endif //DEBUG

      while (detectKey(DOWN, LOW, WAIT) == LOW) {}

#ifdef DEBUG
      _INFOLIST("%s Key DOWN released\n", __func__);
#endif //DEBUG

      Band_slot = changeBand(+1);
      setLED(LED[3 - Band_slot], true);

#ifdef DEBUG
      _INFOLIST("%s slot(%d)\n", __func__, Band_slot);
#endif //DEBUG

    }
    if (detectKey(TXSW, LOW, NOWAIT) == LOW) {

#ifdef DEBUG
      _INFOLIST("%s Key TX detected\n", __func__);
#endif //DEBUG

      while (detectKey(TXSW, LOW, WAIT) == LOW) {}

#ifdef DEBUG
      _INFOLIST("%s Key TX released\n", __func__);
#endif //DEBUG

      setGPIO(TX, LOW);

#ifdef RESET
      uint32_t tnow = millis();
      while (getTXSW() == LOW) {
        if (millis() - tnow > RTIME) {
          setLED(WSPR, false);
          setLED(JS8, false);
          setLED(FT4, false); //@@@ Correct , by ;
          setLED(FT8, false); //@@@ Correct , by ;
          delay(500);         //@@@ Correct , by ;
          resetLED();
          delay(500);
        }
#ifdef WDT
        wdt_reset();
#endif //WDT
      }
      if (millis() - tnow > RTIME) {
        resetFunc();
      }
#endif //RESET

   
#ifdef EE
      flagEEPROM();
#endif //EE

      setLED(TX, false);
      Band_assign(true);
      resetBand(Band_slot);
      /*------------------------------------------*
         Update master frequency
        ------------------------------------------*/
      return;
    }
  }
}

/*---------------------------------------------------------------------------*
    checkMode
    manage change in mode during run-time
  ---------------------------------------------------------------------------*/
void checkMode() {
  /*--------------------------------*
     TX button short press
     Transmit mode
    --------------------------------*/
      
  if ((detectKey(TXSW, LOW, NOWAIT) == LOW) && (getWord(SSW, TXON) == false)) {

     #ifdef DEBUG
       _INFOLIST("%s TX+\n", __func__);
     #endif //DEBUG

     ManualTX();

     #ifdef DEBUG
       _INFOLIST("%s TX-\n", __func__);
     #endif //DEBUG
  }

  /*-------------------------------------------------------------*
     manage band, mode and  calibration selections
    -------------------------------------------------------------*/
#ifndef ONEBAND

  if ((detectKey(UP, LOW, NOWAIT) == LOW) && (detectKey(DOWN, LOW, NOWAIT) == LOW) && (getWord(SSW, TXON) == false)) {
    while (detectKey(UP, LOW, NOWAIT) == LOW) {}
    while (detectKey(DOWN, LOW, NOWAIT) == LOW) {} //Wait till both switches are up

    Band_Select();

    #ifdef DEBUG
       _INFOLIST("%s U+D f=%ld", __func__, freq);
    #endif //DEBUG
  }

#endif //ONEBAND

  if ((detectKey(DOWN, LOW, NOWAIT) == LOW) && (getWord(SSW, TXON) == false)) {


    while (detectKey(DOWN, LOW, NOWAIT) == LOW) {}

    if (mode == 0) {
#ifdef CW
      mode = MAXMODE - 1;
#else
      mode = MAXMODE - 2;
#endif //CW
    } else {
      mode--;
    }

#ifdef DEBUG
    _INFOLIST("%s m+(%d)\n", __func__, mode);
#endif //DEBUG

#ifdef EE
    flagEEPROM();
#endif //EE

    Mode_assign();

#ifdef DEBUG
    _INFOLIST("%s mode assigned(%d)\n", __func__, mode);
#endif //DEBUG

  }

  if ((detectKey(UP, LOW, NOWAIT) == LOW) && (getWord(SSW, TXON) == false)) {

    while (detectKey(UP, LOW, NOWAIT) == LOW) {}

    mode++;
    /*---
       CW enables a 5th mode
    */
#ifdef CW
    if (mode == MAXMODE - 1) {
      setWord(&SSW, CWMODE, true);
    } else {
      setWord(&SSW, CWMODE, false);
    }
    if (mode > MAXMODE - 1) {
      mode = 0;
    }
#else
    if (mode > MAXMODE - 2) {
      mode = 0;
    }
    setWord(&SSW, CWMODE, false);
#endif //CW

#ifdef EE
    flagEEPROM();
#endif //EE Avoid the tear and wear of the EEPROM because of successive changes

#ifdef DEBUG
    _INFOLIST("%s m-(%d)\n", __func__, mode);
#endif //DEBUG

    Mode_assign();

#ifdef DEBUG
    _INFOLIST("%s mode assigned(%d)\n", __func__, mode);
#endif //DEBUG

  }
}
/*---------------------------------------------------------------------------------*
   keepAlive()
   Reference function for debugging purposes, called once per loop
  ---------------------------------------------------------------------------------*/
void keepAlive() {

#ifdef DEBUG

  /* Code here -- Reserved for future usage -- hook into loop*/

#endif //DEBUG

}


//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                   Board Initialization and Support                                          *
//* Perform all the functions to initialize the default operation                               *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*

/**********************************************************************************************/
/*                            Initialization and Setup                                        */
/**********************************************************************************************/

/*----------------------------------------------------------*
   Initialization function from EEPROM
  ----------------------------------------------------------*/
void initTransceiver() {


#ifdef EE
  initEEPROM();
#endif // EE

#ifdef DEBUG
  _INFOLIST("%s entering setup_si5351()\n", __func__, Band_slot);
#endif //DEBUG

  setup_si5351();

#ifdef DEBUG
  _INFOLIST("%s entering resetBand(%d)\n", __func__, Band_slot);
#endif //DEBUG

  resetBand(Band_slot);  //@@@

#ifdef DEBUG
  _INFOLIST("%s completed resetBand(%d)\n", __func__, Band_slot);
#endif //DEBUG


  switch_RXTX(LOW);   //Turn-off transmitter, establish RX LOW
  delay(100);

#ifdef DEBUG
  _INFOLIST("%s setup m(%d) slot(%d) f(%ld)\n", __func__, mode, Band_slot, freq);
#endif //DEBUG
}
/*--------------------------------------------------------------------------*
   definePinOut
   isolate pin definition on a board conditional procedure out of the main
   setup flow
  --------------------------------------------------------------------------*/
void definePinOut() {

  gpio_init(TX);
  gpio_init(UP);
  gpio_init(DOWN);
  gpio_init(TXSW);
  gpio_init(RX);
  gpio_init(WSPR);
  gpio_init(JS8);
  gpio_init(FT4);
  gpio_init(FT8);
  gpio_init(FSK);


  gpio_set_dir(UP, GPIO_IN);
  gpio_set_dir(DOWN, GPIO_IN);
  gpio_set_dir(TXSW, GPIO_IN);

  gpio_pull_up(TXSW);
  gpio_pull_up(DOWN);
  gpio_pull_up(UP);

  gpio_set_dir(RX, GPIO_OUT);
  gpio_set_dir(TX, GPIO_OUT);
  gpio_set_dir(WSPR, GPIO_OUT);
  gpio_set_dir(JS8, GPIO_OUT);
  gpio_set_dir(FT4, GPIO_OUT);
  gpio_set_dir(FT8, GPIO_OUT);

  gpio_set_dir(FSK, GPIO_IN);

  #ifdef ATUCTL
     initATU();
  #endif //ATUCTL   


  Wire.setSDA(PDX_I2C_SDA);
  Wire.setSCL(PDX_I2C_SCL);
  Wire.begin();

#ifdef DEBUG
  _INFOLIST("%s completed\n", __func__);
#endif //DEBUG

}
/*---------------------------------------------------------------------------------------------
   setup()
   This is the main setup cycle executed once on the Arduino architecture
  ---------------------------------------------------------------------------------------------*/
void setup()
{
  /*-----
     Initialization is common for all uses of the serial port, specific variables and constants
     has been given proper initialization based on the protocol used
    -----*/

//#define HAVE_USB_PATCHES 1
#ifdef HAVE_USB_PATCHES
  Serial.ignoreDTR();
#endif

//#if defined(DEBUG) || defined(TERMINAL)
  Serial.begin(BAUD);
  while (!Serial);
//#endif //DEBUG
  
  Serial1.setTX(UART_TX);
  Serial1.setRX(UART_RX);  
  Serial1.begin(BAUD);

#ifdef DEBUG
  const char * proc = "RP2040";
  _INFOLIST("%s: PDX Firmware V(%s) build(%d) board(%s)\n", __func__, VERSION, BUILD, proc);
  sprintf(hi,"%s: PDX Firmware V(%s) build(%d) board(%s)\n", __func__, VERSION, BUILD, proc);
  Serial1.write(hi);
  Serial.write(hi);
#endif //DEBUG

  /*---
     List firmware properties at run time
  */
#ifdef DEBUG

#ifdef EE
  _INFOLIST("%s EEPROM Sub-system activated\n", __func__);
#endif //EE

#ifdef WDT
  _INFOLIST("%s Watchdog Sub-system activated\n", __func__);
#endif //WDT

#ifdef TERMINAL
  _INFOLIST("%s Terminal Sub-system activated\n", __func__);
#endif //TERMINAL

#ifdef RESET
  _INFOLIST("%s Reset feature activated\n", __func__);
#endif //TERMINAL

#ifdef ATUCTL
  _INFOLIST("%s ATU Reset Sub-system activated\n", __func__);
#endif //ATUCTL

#ifdef ONEBAND
  _INFOLIST("%s ONE BAND feature activated Band[%d]\n", __func__,Bands[0]);
#else
  _INFOLIST("%s MULTI BAND feature activated Bands[%d,%d,%d,%d]\n", __func__,Bands[0],Bands[1],Bands[2],Bands[3]);
#endif //ONEBAND

#ifdef QUAD
  _INFOLIST("%s Quad Band filter support activated\n", __func__);
#endif //ONEBAND

#ifdef FSK_PEG
  _INFOLIST("%s PEG decoding algorithm used Mult(%d) Window[uSec]=%d \n", __func__, uint16_t(FSK_MULT), uint16_t(FSK_WINDOW_USEC));
#endif //ONEBAND

#ifdef FSK_ZCD
  _INFOLIST("%s ZCD decoding algorithm used (@core2)\n", __func__);
#endif //FSK_ZCD

#ifdef FSK_ADCZ
  _INFOLIST("%s ACDZ decoding algorithm used (@core2)\n", __func__);
#endif //FSK_ADCZ

#ifdef FSK_FREQPIO
  _INFOLIST("%s FREQPIO decoding algorithm used (@core1)\n", __func__);
#endif //FREQPIO

#endif //DEBUG

#ifdef WIFI
  init_WiFi();
#ifdef DEBUG
  _INFOLIST("%s WiFi connectivity triggered\n", __func__);
#endif //DEBUG
#endif //WIFI


  definePinOut();

#ifdef DEBUG
  _INFOLIST("%s definePinOut() ok\n", __func__);
#endif //DEBUG

  blinkLED(TX);

#ifdef DEBUG
  _INFOLIST("%s blink LED blink dance Ok\n", __func__);
#endif //DEBUG

#ifdef DEBUG
  _INFOLIST("%s setup_si5351 ok\n", __func__);
#endif //DEBUG

  initTransceiver();

#ifdef DEBUG
  _INFOLIST("%s initTransceiver ok\n", __func__);
#endif //DEBUG

  /*------
    Check if calibration is needed
  */

  if (detectKey(DOWN, LOW, WAIT) == LOW) {

#ifdef DEBUG
    _INFOLIST("%s Calibration mode detected\n", __func__);
#endif //DEBUG

    setWord(&QSW, QCAL, true);
    setWord(&QSW, QWAIT, true);
    while (true) {
#ifdef WDT
      wdt_reset();
#endif //WDT
    }
  }

#ifdef DEBUG
  _INFOLIST("%s calibration mode not required\n", __func__);
#endif //DEBUG


  // trigger counting algorithm on the second core
  rp2040.idleOtherCore();

#ifdef DEBUG
  _INFOLIST("%s Core1 stopped ok\n", __func__);
#endif //DEBUG

  setWord(&QSW, QFSK, true);
  setWord(&QSW, QWAIT, true);

#if defined (FSK_ZCD) || defined(FSK_ADCZ)
#ifdef DEBUG
  _INFOLIST("%s FSK detection algorithm started QFSK=%s QWAIT=%s ok\n", __func__, BOOL2CHAR(getWord(QSW, QFSK)), BOOL2CHAR(getWord(QSW, QWAIT)));
#endif //DEBUG
#endif //FSK_ZCD or FSK_ADCZ, otherwise core2 won't be activated and it will sit idle on loop1()

/*-----
 * If the PIO based counting algorithm is activated the whole PIO setup
 * is performed
 */
#ifdef FSK_FREQPIO

  PIO_init();
  
#ifdef DEBUG
  _INFOLIST("%s PIO firmware loaded and sm setup\n", __func__);
#endif //DEBUG 

#endif //FSK_FREQPIO

  delay(500);
  switch_RXTX(LOW);

#ifdef DEBUG
  _INFOLIST("%s switch_RXTX Low ok\n", __func__);
#endif //DEBUG

#ifdef WDT
  watchdog_enable(8000, 1);
  setWord(&TSW, TX_WDT, false);
#ifdef DEBUG
  _INFOLIST("%s watchdog configuration completed\n", __func__);
#endif //DEBUG
#endif //WDT

#ifdef TERMINAL
  /*------------------
     if UP switch pressed at bootup then enter Terminal mode
  */
  if (detectKey(UP, LOW, WAIT) == LOW) {
    execTerminal();
  }
#endif //TERMINAL


  /*------------
     re-start the core1 where the FSK counting or ADC zero crossing algorithms
     are used
  */
  rp2040.restartCore1();
  delay(1);

#ifdef DEBUG
  _INFOLIST("%s Core1 resumed ok\n", __func__);
#endif //DEBUG



#ifdef DEBUG
  _INFOLIST("%s setup configuration completed\n", __func__);
#endif //DEBUG

}
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                   Board Main Dispatched and operational loop                                *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
void loop()
{

  //*--- Debug hook
  keepAlive();

  /*---------------------------------------------------------------------------------*
      Manage the user interface (UP/DOWN/TX switches & combinations)
      Change frequency, mode, band
    ---------------------------------------------------------------------------------*/
  checkMode();

  /*---------------------------------------------------------------------------------*
      Save EEPROM if a change has been flagged anywhere in the logic
    ---------------------------------------------------------------------------------*/
#ifdef EE
  //*--- if EEPROM enabled check if timeout to write has been elapsed
  checkEEPROM();
#endif //EEPROM

  /*---------------------------------------------------------------------------------*
      ATU pulse width control, reset signal after the elapsed time elapsed happens
    ---------------------------------------------------------------------------------*/
#ifdef ATUCTL
  checkATU();
#endif //ATUCTL

  /*---------------------------------------------------------------------------------*
      Sample for CAT commands if enabled
    ---------------------------------------------------------------------------------*/
#ifdef CAT
  serialEvent();
#endif //CAT

#ifdef WDT
  if ((millis() > (wdt_tout + uint32_t(WDT_MAX))) && getWord(SSW, TXON) == HIGH && getWord(SSW, CATTX) == true) {
    switch_RXTX(LOW);
    setWord(&TSW, TX_WDT, HIGH);
    wdt_tout = millis();
  }

  //*--- if WDT enabled reset the watchdog
  wdt_reset();
#endif //WDT
/*---------------------------------------------------------------------------------*
 * Manage WiFi connection status while not transmitting
 *---------------------------------------------------------------------------------*/
#ifdef WIFI
  check_WiFi();
#endif //WIFI

  //=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
  //*                                                                                *
  //*                      PDX Counting Algorithm                                    *
  //*                                                                                *
  //=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
  /*---------------------------------------------------------------------------------*
     The counting algorithm runs either at core1 (ZCD & ADCZ) or on a PIO RISC (FREQPIO)
     in the case of core1 the algorithms runs at setup1() and sampling the frequency
     using either a pwm counting (FSK_PEG, deprecated) or a pseudo zero crossing (FSK_ZCD)
     method (digital) or the ADCZ zero cross detector (analog).
     Whenever the frequency falls within the [FSKMIN,FSKMAX] limits it's FIFOed here
     Additional heuristic of validation are also applied to manage counting common
     counting errors.
     FSK_PEG (deprecated, way too much error)
     The counting algorithm has a common error of +/- 1 count because of the moment
     the sampling starts (which might include or exclude one edge), because of the
     window measurement applied this is translated into a +/- FSK_MULT (Hz) error
     This value needs to be much larger than the maximum bandwidth of the signal to
     be decoded. i.e. With FSK_WINDOW at 10 mSec FSK_MULT is 100 thus the count
     error can be up to +/- 100 Hz. As the FT8 signal occupies up to 50 Hz then the
     deviation is produced by a counting common error and not of a PSK tone change
     and thus ignored. Other counting errors can produce an actual shift of the
     transmitting frequency and thus a decoding issue on the other side. Actual
     measurement seems to point to error<0.2%
     FSK_ZCD (adopted, requires a comparator/shaper)
     The counting algorithm has rounding errors in the range of <500 uSec because the
     error in the triggering level and the residual +/- 1 uSec counting error
     the frequency is filtered by the bandwidth level and also a rounding error of
     1 Hz, thus if the sampled frequency is off by +/- 1 Hz the difference is less
     than the change on the PSK tone and thus ignored, further filtering is made
     by not allowing the frequency to change if below a threshold.
     ADCZ works on a different ground as well as FREQPIO
    ---------------------------------------------------------------------------------*/
  uint16_t n = vox_maxtry;
  
  setWord(&SSW, VOX, false);
  int k=0;
  while ( n > 0 ) {                                //Iterate up to 10 times looking for signal to transmit

/*----
 * Follows a section not related to the actual detection but to the care of a racing condition
 * leaving the TX turned on. To avoid this situation to result in a large transmission period
 * with potential damage to the finals the watchdog is used to prevent that
 */
#ifdef WDT
    wdt_reset();

    if (getWord(TSW, TX_WDT) == HIGH) {
      break;
    }  //If watchdog has been triggered so no TX is allowed till a wdt_max timeout period has elapsed

    if ((millis() > (wdt_tout + uint32_t(WDT_MAX))) && getWord(SSW, TXON) == HIGH) {
      switch_RXTX(LOW);
      setWord(&TSW, TX_WDT, HIGH);
      wdt_tout = millis();
#ifdef DEBUG
      _INFOLIST("%s TX watchdog condition triggered\n", __func__);
#endif //DEBUG
      break;
    }
#endif //WDT


#if defined(FSK_ADCZ) || defined(FSK_ZCD)
/*----------------------------------------------------------------------------------------------*
 * The core2 is running the counting algorithms (either ZCD or ADCZ) and pushing results thru   *
 * the IPC FIFO for the main process at loop() to get the frequencies and process them          *
 * The following code segregates the code for this case                                         *
 * ============================================================================================ *
    /*-----------------------------------------------------
       frequency measurements are pushed from core1 when
       a sample is available. If no signal is available
       no sample is provided. Thus it's wait for a number
       of cycles till extingish the TX mode and fallback
       into RX mode.
      -----------------------------------------------------*/
    if (rp2040.fifo.available() != 0) {

#ifdef FSK_ZCD
      int index = rp2040.fifo.pop();
      double fo = fsequences[index];
      codefreq = uint32_t(round(fo));
      n = vox_maxtry;
#endif //FSK_ZCD

#ifdef FSK_ADCZ
      codefreq = rp2040.fifo.pop();
      n = vox_maxtry;
#endif //FSK_ADCZ

#ifdef DEBUG
      _INFOLIST("%s FIFO f=%ld\n", __func__, codefreq);
#endif //DEBUG

      /*------------------------------------------------------*
         Filter out frequencies outside the allowed bandwidth
        ------------------------------------------------------*/
      if (codefreq >= uint32_t(FSKMIN) && codefreq <= uint32_t(FSKMAX)) {
        n = vox_maxtry;

        /*----------------------------------------------------*
           if VOX is off then pass into TX mode
           Frequency IS NOT changed on the first sample
          ----------------------------------------------------*/

        if (getWord(SSW, VOX) == false) {
#ifdef DEBUG
          _INFOLIST("%s VOX activated n=%d f=%ld\n", __func__, n, codefreq);
#endif //DEBUG
          switch_RXTX(HIGH);
          setWord(&SSW, VOX, true);
          prevfreq = 0;
          n = vox_maxtry;
          continue;
        }
        /*-----------------------------------------------------*
           If this is the first sample AFTER the one that set
           the VOX on then switch the frequency to it
          -----------------------------------------------------*/
#ifdef FSK_ZCD
        if (pfo != codefreq) {
          si5351.set_freq(((freq + codefreq) * 100ULL), SI5351_CLK0);
#ifdef DEBUG
          _INFOLIST("%s Frequency change fo=%ld\n", __func__, codefreq);
#endif //DEBUG
          pfo = codefreq;
          continue;
        }
#endif //PSK_ZCD

#ifdef FSK_ADCZ
        if (codefreq != prevfreq) {
          si5351.set_freq(((freq + codefreq) * 100ULL), SI5351_CLK0);
#ifdef DEBUG
          _INFOLIST("%s Frequency change f=%ld prev=%ld\n", __func__, codefreq, prevfreq);
#endif //DEBUG
          prevfreq = codefreq;
        }
#endif //FSK_ADCZ
      }
      /*----------------
         Watchdog reset
        ---------------*/
#ifdef WDT
      wdt_reset();
#endif //WDT
}
#endif //End the processing made by the ADCZ and ZCD algorithms    
//*===================================================================================================================    

#if defined(FSK_FREQPIO)
/*--------------
 * Just wait for IRQ request made by the PIO sequential machine signaling
 * a complete cycle measurement. Each time a full cycle has been measured
 * the result is communicated and can be used
 */

if (getWord(QSW,PIOIRQ) == true) {
   setWord(&QSW,PIOIRQ,false); 
   n=vox_maxtry;
   if (period>0) {             
      codefreq=FSK_USEC/period;
   } else {
      codefreq=0;   
   }
   /*------------------------------------------------------*
    Filter out frequencies outside the allowed bandwidth
    ------------------------------------------------------*/
   if (codefreq >= uint32_t(FSKMIN) && codefreq <= uint32_t(FSKMAX)) {
    
      n = vox_maxtry;

      /*----------------------------------------------------*
       if VOX is off then pass into TX mode
       Frequency IS NOT changed on the first sample
       ----------------------------------------------------*/
      if (getWord(SSW, VOX) == false) {
#ifdef DEBUG
         _INFOLIST("%s VOX activated n=%d f=%ld\n", __func__, n, codefreq);
#endif //DEBUG
          switch_RXTX(HIGH);
          setWord(&SSW, VOX, true);
          prevfreq = 0;
          n = vox_maxtry;
          continue;
      }
      /*-----------------------------------------------------
       * Avoid producing jitter by changing the frequency 
       * by less than 4 Hz.
       */
      if (abs(int(codefreq-prevfreq))>=FSK_ERROR) {
         si5351.set_freq(((freq + codefreq) * 100ULL), SI5351_CLK0);
#ifdef DEBUG
         k++;
         if (k=1000) {
         _INFOLIST("%s T=%ld fsk=%ld fskprev=%ld\n", __func__, period,codefreq, prevfreq);
         k=0;
         }
#endif //DEBUG
         prevfreq = codefreq;
      }
   }        
}

#endif //End the processing made by the FSK_FREQPIO algorithms    
//*===================================================================================================================    
    
    
    else {   //within the nth loop but no signal is detected, so it's a waiting pattern to turn off the VOX or CAT
      /*--------------------
         Waiting for signal
        --------------------*/
      uint32_t tcnt = time_us_32() + uint32_t(FSK_IDLE);
      while (time_us_32() < tcnt);
      n--;
#ifdef WDT
      wdt_reset();
#endif //WDT

#ifdef CAT
      serialEvent();
#endif //CAT
    }

    /*----------------------*
       Sample CAT commands
      ----------------------*/
#ifdef CAT
    serialEvent();
#endif //CAT

    /*----------------------*
       Sample watchdog reset
      ----------------------*/
#ifdef WDT
    wdt_reset();
#endif //WDT

  } //This is the anchor footer of the nth loop waiting for signal to appear below here is processed when no signal is present

  /*---------------------------------------------------------------------------------*
     when out of the loop no further TX activity is performed, therefore the TX is
     turned off and the board is set into RX mode
    ---------------------------------------------------------------------------------*/
  //=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
  //*                               RX Cycle                                               *
  //=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
  /*---------------------------------------------------------------------------------------*
     TX cycle ends, fallback to RX mode
    ---------------------------------------------------------------------------------------*/

#ifdef WDT
  /*-----------------------------------------------------------*
     Check for watchdog
     if activated blink TX LED and wait till a full timeout to
     restore the TX capability.
     If the continuous TX condition persists (i.e. noise at the
     SPKR input at least some cooling time was allowed, if
     the watchdog was activated by a wrong CAT command then
     it will stay ready for the next TX command
   *                                                           *
    -----------------------------------------------------------*/
  if (getWord(TSW, TX_WDT) == HIGH) {
    blinkLED(TX);
  }

  if (getWord(SSW, TXON) == LOW && getWord(TSW, TX_WDT) == HIGH && (millis() > (wdt_tout + uint32_t(WDT_MAX)))) {
    setWord(&TSW, TX_WDT, LOW); //Clear watchdog condition
    #ifdef DEBUG
       _INFOLIST("%s TX watchdog condition cleared\n", __func__);
    #endif //DEBUG

    /*-----
       Clear FIFO
      -----*/
    while (rp2040.fifo.available() != 0) {
      uint32_t dummy = rp2040.fifo.pop();
    }
  }
#endif //WDT

  /*----------------------*
     Sample CAT commands
    ----------------------*/
#ifdef CAT
  serialEvent();
#endif //CAT

  /*------------------------------------------------------------*
     At this point it must be in RX mode so perform the switch
    ------------------------------------------------------------*/
  if (getWord(SSW, CATTX) != true) {
    switch_RXTX(LOW);
    setWord(&SSW, VOX, false);
    setWord(&SSW, TXON, false);
    pfo = 0;                         //Force next TX mode to setup frequency
  }

  /*----------------------*
     Reset watchdog
    ----------------------*/
#ifdef WDT
  wdt_reset();
#endif //WDT

}
//****************************[ END OF MAIN LOOP FUNCTION ]*******************************
//********************************[ END OF FIRMWARE ]*************************************
