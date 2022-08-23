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
#include <si5351.h>
#include "pdx_common.h"

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                            VERSION HEADER                                                   *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
#define VERSION        "2.0"
#define BUILD          3

#ifdef TERMINAL
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*               DEFINITIONS SPECIFIC TO THE CONFIGURATION TERMINAL FUNCTION                   *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
#include <string.h>
#include <stdlib.h>
#define  CR '\r'
#define  LF '\n'
#define  BS '\b'
#define  NULLCHAR '\0'
#define  SPACE ' '

#define  COMMAND_BUFFER_LENGTH        25                     //length of serial buffer for incoming commands
char     cmdLine[COMMAND_BUFFER_LENGTH + 1];                 //Read commands into this buffer from Serial.  +1 in length for a termination char

const char *delimiters            = ", \n";                  //commands can be separated by return, space or comma

/*----------------------------------------------------------*
   Serial configuration terminal commands
  ----------------------------------------------------------*/
#ifdef ATUCTL
const char *atuToken        = "*atu";
const char *atu_delayToken  = "*atd";
#endif //ATUCTL

const char *bounce_timeToken = "*bt";
const char *short_timeToken = "*st";
const char *max_blinkToken  = "*mbl";


#ifdef EE
const char *eeprom_toutToken = "*eet";
const char *eeprom_listToken = "*list";
#endif //EE


const char *saveToken       = "*save";
const char *quitToken       = "*quit";
const char *resetToken      = "*reset";
const char *helpToken       = "*help";
const char *endList         = "XXX";

#endif //TERMINAL

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


#ifdef ATUCTL
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*               ATU RESET FUNCTION SUPPORT                                                    *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
#define ATU_DELAY    200       //How long the ATU control line (D5) is held HIGH on band changes, in mSecs

uint16_t atu       =  ATU;
uint16_t atu_delay =  ATU_DELAY;
uint32_t tATU = 0;

#endif //ATUCTL


#ifdef EE
uint32_t tout = 0;
#endif //EE


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

double fsequences[NFS]; // Ring buffer for communication across cores
int nfsi   = 0;
double pfo = 0; // Previous output frequency


//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*       GLOBAL VARIABLE DEFINITION CONDITIIONAL TO DIFFERENT OPTIONAL FUNCTIONS               *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*

#ifdef EE
uint16_t eeprom_tout = EEPROM_TOUT;
#endif //EE

#if (defined(ATUCTL) || defined(WDT))
#endif //Either ATU or WDT has been defined

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

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*==*
//*                    ATU DEVICE MANAGEMENT                                                     *
//*this is an optional function that creates a brief pulse aimed to reset the ATU on band changes*
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
#ifdef ATUCTL
void flipATU() {

  setGPIO(atu, HIGH);
  setWord(&TSW, ATUCLK, true);
  tATU = millis();

#ifdef DEBUG
  _INFO;
#endif //DEBUG
}
#endif //ATUCTL

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

  if (LEDpin == uint8_t(TX)) {
    setGPIO(LED_BUILTIN, LOW);
  }

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
  clearLED(LED_BUILTIN);

#ifdef DEBUG
  _INFO;
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

  if (LEDpin == uint8_t(TX)) {
    setGPIO(LED_BUILTIN, HIGH);
  }

#ifdef DEBUG
  _INFOLIST("%s(%d)\n", __func__, LEDpin);
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

    if (LEDpin == uint8_t(TX)) {
      setGPIO(LED_BUILTIN, HIGH);
    }

    delay(BDLY);
    setGPIO(LEDpin, LOW);
    if (LEDpin == uint8_t(TX)) {
      setGPIO(LED_BUILTIN, LOW);
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
    setGPIO(LED_BUILTIN, HIGH);
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
  setGPIO(LED_BUILTIN, LOW);
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

/*---------------------------------------------------------------------*
   getSwitchPL
   Detect and clear the Long push condition on both UP/DOWN buttons
  ---------------------------------------------------------------------*/
bool getSwitchPL(uint8_t pin) {
  return HIGH;
}

/*----------------------------------------------------------*
   get value for a digital pin and return after debouncing
  ----------------------------------------------------------*/
bool getSwitch(uint8_t pin) {
  return detectKey(pin, LOW, WAIT);
}

/*----------------------------------------------------------*
   read UP switch
  ----------------------------------------------------------*/
bool getUPSSW() {

  return getSwitch(UP);

}

/*----------------------------------------------------------*
   read DOWN Switch
  ----------------------------------------------------------*/
bool getDOWNSSW() {
  return getSwitch(DOWN);
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

//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*                           PDX Calibration (automatic) and FSK counting algorithm                        *
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
/*------
   Înterrupt IRQ for edge counting overflow
  -----*/
void pwm_int() {
  pwm_clear_irq(pwm_slice);
  f_hi++;
}

#ifdef FSK_ADCZ
/*------------------------------------------------------------------------------------------*
   calibrateADC
   Calibrate the ADC zero level
*/
uint16_t calibrateADC(uint16_t min, uint16_t max) {
  return uint16_t((adc_max - adc_min) * 1.0 / 2.0) + adc_min;
}

/*-------------------------------------------------------------------------------------------*
   ADCreset
   restore all calibration values
*/
void ADCreset() {
  adc_min = ADCMAX;
  adc_max = ADCMIN;
  adc_zero = ADCZERO;
  adc_uh = adc_zero * 110 / 100;
  adc_ul = adc_zero * 90 / 100;
  ffmin = FSKMAX;
  ffmax = FSKMIN;
#ifdef DEBUG
  _TRACELIST("%s Timeout break QSTATE=0, recalibrate input level", __func__);
#endif //DEBUG
}

/*------------------------------------------------------------------------------------------*
   getADCsample
   collect an ADC sample running free.
   update the minimum and maximum
*/
uint16_t getADCsample() {
  uint16_t v = adc_read();
  if (v > adc_max) {
    adc_max = v;
    adc_zero = calibrateADC(adc_min, adc_max);
    adc_uh = adc_zero * 110 / 100;
    adc_ul = adc_zero * 90 / 100;
#ifdef DEBUG
    _TRACELIST("%s calibration (max) adc_max=%d adc_min=%d adc_Zero=%d\n", __func__, adc_max, adc_min, adc_zero);
#endif //DEBUG
    return v;
  }
  if (v <= adc_min) {
    adc_min = v;
    adc_zero = calibrateADC(adc_min, adc_max);
    adc_uh = adc_zero * 110 / 100;
    adc_ul = adc_zero * 90 / 100;
#ifdef DEBUG
    _TRACELIST("%s calibration (min) adc_max=%d adc_min=%d adc_Zero=%d\n", __func__, adc_max, adc_min, adc_zero);
#endif //DEBUG
    return v;
  }
  if (v >= adc_uh) {
    adc_high = true;
  }
  if (v <= adc_ul) {
    adc_low = true;
  }

  return  v;
}
#endif //FSK_ADCZ

/*=========================================================================================*
   CORE1
   2nd rp2040 core instantiated by defining setup1/proc1 procedures
   These procedures are used to run frequency measurement / time sensitive code
   the por1 procedure isn't never reached actually as the flow is left at an infinite loop
   at setup1
  =========================================================================================*/
void setup1() {
  /*-----------------------------------------------------------------*
     Core1   Setup procedure
     Enter processing on POR but restarted from core0 setup ()
    -----------------------------------------------------------------*/
  uint32_t t = 0;
  bool     b = false;
  /*--------------------------------------------*
     Wait for overall initialization to complete
    --------------------------------------------*/
  while (getWord(QSW, QWAIT) == false) {

#ifdef WDT
    wdt_reset();
#endif //WDT

    uint32_t t = time_us_32() + 2;
    while (t > time_us_32());
  }
  /*-------------------------------------------*
     Semaphore QWAIT has been cleared, proceed
     PWM counters operates as infinite loops
     therefore no loop1() is ever processed
    -------------------------------------------*/
#ifdef DEBUG
  _INFOLIST("%s Core1 waiting semaphore released QCAL=%s QFSK=%s\n", __func__, BOOL2CHAR(QCAL), BOOL2CHAR(QFSK));
#endif //DEBUG

  //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
  //* Automatic calibration procedure                                                                             *
  //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
  if (getWord(QSW, QCAL) == true) {
#ifdef DEBUG
    _INFOLIST("%s Calibration procedure triggered\n", __func__);
#endif //DEBUG
    delay(1000);
    calibrateLED();

    /*----
       Prepare Si5351 CLK2 for calibration process
      ---*/
#ifdef DEBUG
    _INFOLIST("%s Automatic calibration procedure started\n", __func__);
#endif //DEBUG

    switch_RXTX(LOW);

    gpio_set_function(CAL, GPIO_FUNC_PWM); // GP9
    si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA); // Set for lower power for calibration
    si5351.set_clock_pwr(SI5351_CLK0, 0); // Enable the clock for calibration
    si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA); // Set for lower power for calibration
    si5351.set_clock_pwr(SI5351_CLK1, 0); // Enable the clock for calibration
    si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA); // Set for lower power for calibration
    si5351.set_clock_pwr(SI5351_CLK2, 1); // Enable the clock for calibration
    si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
    si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
    si5351.set_freq(Cal_freq * 100UL, SI5351_CLK2);

    /*--------------------------------------------*
       PWM counter used for automatic calibration
       -------------------------------------------*/
    fclk = 0;
    int16_t n = int16_t(CAL_COMMIT);
    cal_factor = 0;
#ifdef DEBUG
    _INFOLIST("%s si5351 initialization ok target freq=%ld cal_factor=%ld\n", __func__, Cal_freq, cal_factor);
#endif //DEBUG

    pwm_slice = pwm_gpio_to_slice_num(CAL);
    while (true) {
      /*-------------------------*
         setup PWM counter
        -------------------------*/
      pwm_config cfg = pwm_get_default_config();
      pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_RISING);
      pwm_init(pwm_slice, &cfg, false);
      gpio_set_function(CAL, GPIO_FUNC_PWM);

      pwm_set_irq_enabled(pwm_slice, true);
      irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_int);
      irq_set_enabled(PWM_IRQ_WRAP, true);
      f_hi = 0;

      /*---------------------------*
         PWM counted during 1 sec
        ---------------------------*/
      t = time_us_32() + 2;
      while (t > time_us_32());
      pwm_set_enabled(pwm_slice, true);
      t += 1000000;
      while (t > time_us_32());
      pwm_set_enabled(pwm_slice, false);

      /*----------------------------*
         recover frequency in Hz
        ----------------------------*/
      fclk = pwm_get_counter(pwm_slice);
      fclk += f_hi << 16;
      error = fclk - Cal_freq;

#ifdef DEBUG
      _INFOLIST("%s Calibration VFO=%ld Hz target_freq=%ld error=%ld cal_factor=%ld\n", __func__, fclk, Cal_freq, error, cal_factor);
#endif //DEBUG

      if (labs(error) > int32_t(CAL_ERROR)) {
        b = !b;
        if (b) {
          setLED(TX, false);
        } else {
          rstLED(TX, false);
        }
        if (error < 0) {
          cal_factor = cal_factor - CAL_STEP;
        } else {
          cal_factor = cal_factor + CAL_STEP;
        }
        si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
      } else {
        n--;
        if (n == 0) {

#ifdef DEBUG
          _INFOLIST("%s Convergence achieved cal_factor=%ld\n", __func__, cal_factor);
#endif //DEBUG

#ifdef EE
          updateEEPROM();
#endif //EE

          while (true) {
#ifdef WDT
            wdt_reset();
#endif //WDT
#ifdef EE
            checkEEPROM();
#endif //EE
          }
          while (true) {
            resetLED();
            setLED(JS8, true);
            setLED(FT4, false);
            delay(1000);
          }
        }
      }
    }
  } //Auto calibration mode

  //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
  //* FSK detection algorithm                                                                                     *
  //* Automatic input detection algorithm                                                                         *
  //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
  if (getWord(QSW, QFSK) == true) {
    ffsk = 0;

#ifdef FSK_ZCD
    /*----------------------------------------*
       ZCD algorithm
       defined by FSK_ZCD
       this is based on a pseudo cross detect
       where the rising edge is taken as a
       false cross detection followed by next
       edge which is also a false zcd but
       at the same level thus measuring the
       time between both will yield a period
       measurement proportional to the real
       period of the signal as measured
       two sucessive rising edges
       Measurements are made every 1 mSec
      ---------------------------------------*/
    uint16_t cnt = 100;
    pwm_slice = pwm_gpio_to_slice_num(FSK);

#ifdef DEBUG
    _INFOLIST("%s FSK counter() ZCD algorithm triggered\n", __func__);
#endif //DEBUG

    /*--------------------------------------------------------------*
       main counting algorithm cycle
      --------------------------------------------------------------*/
    while (true) {
      pwm_config cfg = pwm_get_default_config();
      pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_RISING);
      pwm_init(pwm_slice, &cfg, false);
      gpio_set_function(FSK, GPIO_FUNC_PWM);
      pwm_set_irq_enabled(pwm_slice, true);
      irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_int);
      irq_set_enabled(PWM_IRQ_WRAP, true);
      f_hi = 0;
#ifdef FSK_ZCD
      uint32_t t = time_us_32() + 2;                     //Allow all the settings to stabilize
      while (t > time_us_32());                          //
      uint16_t j = FSK_RA;                               //
      uint32_t dt = 0;                                    //
      while (j > 0) {                                    //Establish a running average over <j> counts
        uint32_t pwm_cnt = pwm_get_counter(pwm_slice);  //Get current pwm count
        pwm_set_enabled(pwm_slice, true);               //enable pwm count
        while (pwm_get_counter(pwm_slice) == pwm_cnt) {} //Wait till the count change
        pwm_cnt = pwm_get_counter(pwm_slice);           //Measure that value
        uint32_t t1 = time_us_32();                     //Mark first tick (t1)
        while (pwm_get_counter(pwm_slice) == pwm_cnt) {} //Wait till the count change (a rising edge)
        uint32_t t2 = time_us_32();                     //Mark the second tick (t2)
        pwm_set_enabled(pwm_slice, false);              //Disable counting
        dt = dt + (t2 - t1);                            //Add to the RA total
        j--;                                            //Loop
      }                                                  //

      if (dt != 0) {                                     //Prevent noise to trigger a nul measurement
        double dx = 1.0 * dt / double(FSK_RA);          //
        double f = double(FSK_USEC) / dx;               //Ticks are expressed in uSecs so convert to Hz
        double f1 = round(f);                           //Round to the nearest integer
        ffsk = uint32_t(f1);                            //Convert to long integer for actual usage
        if (ffsk >= FSKMIN && ffsk <= FSKMAX) {         //Only yield a value if within the baseband
          fsequences[nfsi] = f;
          rp2040.fifo.push_nb(nfsi);                  //Use the rp2040 FIFO IPC to communicate the new frequency
          nfsi = (nfsi + 1) % NFS;
#ifdef DEBUG
          _TRACELIST("%s dt=%ld dx=%.3f f=%.3f f1=%.3f ffsk=%ld\n", __func__, dt, dx, f, f1, ffsk);
#endif //DEBUG
        }                                               //
      }                                                  //
      t = time_us_32() + FSK_SAMPLE;                     //Now wait for 1 mSec till next sample
      while (t > time_us_32()) ;
#endif //FSK_ZCD
    }  //end FSK (ZCD or PEG) loop
#endif //FSK_ZCD

#ifdef FSK_ADCZ
    /*----------------------------------------*
       ADCZalgorithm
       defined by FSK_ADCZ
       This algorithm samples the ADC port
       at full speed (500 KS/sec) and identify
       zero crossings from + to - values
       Two consecutive epoch are taken and
       the frequency computed from them.
       Actual values obtained from the ADC
       would change based on the VOL setting
       and therefore an adaptive level cal is
       performed. A squelch zone is defined to
       avoid readings with insuficient level
       A direct sampling would take several
       false signals which would lead to false
       freq readings, so an extensive adaptive
       filter is implemented as a finite state
       machine (FSM) in order to ensure the
       consistency of the two epoch taken
      ----------------------------------------*/

#ifdef DEBUG
    _INFOLIST("%s FSK counter() ADCZ algorithm triggered\n", __func__);
#endif //DEBUG

    /*------------------------------------*
       ADC initialization and setup
    */
    adc_init();
    adc_gpio_init( ADC_PIN);
    adc_select_input( ADC_NUM);

    /*----------------------------------------------------*
       Signal processing
       This processing is heavy filtered of inconsistent
       states produced by noise in the signal
       Filtering is performed using a finite state machine
       Wrong intermediate states leading to freq reading
       errors are assumed to be produced randomly and thus
       associated to white noise. The appareance of them
       can be assumed as a hidden system state which needs
       to be filtered. The FSM implemented knows at all
       samples which will the right combination and thus
       is able to predict what is the valid next sample,
       if a different state is observed then it can be
       regarded as noise or corruption of the sample
       stream. Since the algorithm is used to detect
       weak signals which change slowly there is not a
       critical need for samples within the window allowed
       and thus it's safer to drop a corrupt freq reading
       than to keep it and ruin the output stream, then
       the FSM is reset in any detected corrupt state and
       values recalibrated. This corruption can be caused
       by noise but also because the input stream ceased
       and the ADC readings are just the "sound of the
       silence", in this case the detection become silent
       and stop updating the frequency readings to the
       upcall procedure. This behaviour can be associated
       with some liberties to a Kalman filter construct
      ----------------------------------------------------*/

    while (true) {

      switch (QSTATE) {
        //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
        //* State 0 - Wait for the signal to be positive to start a counting cycle       *
        //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
        case 0 : {
            adc_high = false;
            adc_low = false;

            adc_v1 = getADCsample();             //Wait till two sucessive readings are positive, exit if none in 1 mSec
            adc_t1 = time_us_32();
            uint32_t tstop = adc_t1 + 1000;
            while (true) {
              adc_v2 = getADCsample();
              adc_t2 = time_us_32();
              if (adc_v1 >= adc_zero && adc_v2 >= adc_zero) {
                QSTATE = 1;
                break;
              }
              adc_v1 = adc_v2;
              adc_t1 = adc_t2;
              if (time_us_32() > tstop) {
                QSTATE = 0;
                ADCreset();
                break;
              }

            }
          } //Q(0)
        //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
        //* State 1 - Wait for a zero crossing to get the first epoch                    *
        //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
        case 1 : {
            adc_v1 = getADCsample();             //Last state was at least one pair of positive values, so wait for a crossing
            adc_t1 = time_us_32();               //accept but ignore sucessive pairs of positive values, the state looks
            uint32_t tstop = adc_t1 + 1000;      //for one positive and the next negative. Any other combination reset the finite state machine

            while (true) {
              adc_v2 = getADCsample();
              adc_t2 = time_us_32();
              if (adc_v1 >= adc_zero && adc_v2 <= adc_zero) {
                t1[0] = adc_t1;
                t2[0] = adc_t2;
                v1[0] = adc_v1;
                v2[0] = adc_v2;
                QSTATE = 2;
                break;
              }

              if (adc_v1 >= adc_zero && adc_v2 >= adc_zero) {
                adc_v1 = adc_v2;
                adc_t1 = adc_t2;
                if (time_us_32() > tstop) {
                  QSTATE = 0;
#ifdef DEBUG
                  _TRACELIST("%s Break QSTATE=1\n", __func__);
#endif //DEBUG
                  break;
                }
                continue;
              }
#ifdef DEBUG
              _TRACELIST("%s Bad signal QSTATE=1\n", __func__);
#endif //DEBUG
              QSTATE = 0;
              break;


            }

          }  //Q(1)
        //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
        //* State 2 - Wait for the signal to become fully negative                       *
        //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
        case 2 : {                                                //A cross between positive and negative was detected, if another crossing is detected
            adc_v1 = getADCsample();                       //then sampling was fast enough to capture another sucessive crossing, thus the mark
            adc_t1 = time_us_32();                         //is updated. If two sucessive negative values are detected the crossing is completed
            uint32_t tstop = adc_t1 + 1000;                //and the FSM is advanced to next state. Any other combination is weird and reset the FSM

            while (true) {
              adc_v2 = getADCsample();
              adc_t2 = time_us_32();
              if (adc_v1 <= adc_zero && adc_v2 <= adc_zero) {
                QSTATE = 3;
                break;
              }

              if (adc_v1 >= adc_zero && adc_v2 <= adc_zero) {
                t1[0] = adc_t1;
                t2[0] = adc_t2;
                v1[0] = adc_v1;
                v2[0] = adc_v2;

                adc_v1 = adc_v2;
                adc_t1 = adc_t2;
                if (time_us_32() > tstop) {
                  QSTATE = 0;
#ifdef DEBUG
                  _TRACELIST("%s Break QSTATE=2\n", __func__);
#endif //DEBUG
                  break;
                }
                continue;
              }
#ifdef DEBUG
              _TRACELIST("%s Bad signal QSTATE=2\n", __func__);
#endif //DEBUG
              QSTATE = 0;
              break;
            }
          } //Q(2)
        //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
        //* State 3 - Wait for the signal to become fully positive, start 2nd checkpoint *
        //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
        case 3: {                                                  //At this point at least two successive negative values has been obtained
            adc_v1 = getADCsample();                        //the sample stream is now evaluated and any value other than two sucessive positive values
            adc_t1 = time_us_32();                          //is ignored. So when the signal swing back to positive the FSM is advanced.
            uint32_t tstop = adc_t1 + 1000;

            while (true) {
              adc_v2 = adc_read();
              adc_t2 = time_us_32();
              if (adc_v1 >= adc_zero && adc_v2 >= adc_zero) {
                QSTATE = 4;
                break;
              }
              adc_v1 = adc_v2;
              adc_t1 = adc_t2;
              if (time_us_32() > tstop) {
                QSTATE = 0;
#ifdef DEBUG
                _TRACELIST("%s Break QSTATE=3\n", __func__);
#endif //DEBUG
                break;
              }

            }
          } //Q(3)
        //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
        //* State 4 - Wait for the 2nd zero crossing and take the epoch when detected    *
        //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
        case 4: {                                                  //At this point at least two sucessive positive values has been detected. Others might follow
            adc_v1 = getADCsample();                        //which are ignored until another crossing is detected. This is similar to state <1> but for
            adc_t1 = time_us_32();                          //the next cycle. Samples other than both positive or a crossing reset the FSM
            uint32_t tstop = adc_t1 + 1000;

            while (true) {
              adc_v2 = getADCsample();
              adc_t2 = time_us_32();
              if (adc_v1 >= adc_zero && adc_v2 <= adc_zero) {
                t1[1] = adc_t1;
                t2[1] = adc_t2;
                v1[1] = adc_v1;
                v2[1] = adc_v2;
                QSTATE = 5;
                break;
              }
              if (adc_v1 >= adc_zero && adc_v2 >=  adc_zero) {
                adc_v1 = adc_v2;
                adc_t1 = adc_t2;
                if (time_us_32() > tstop) {
                  QSTATE = 0;
#ifdef DEBUG
                  _TRACELIST("%s Break QSTATE=4\n", __func__);
#endif //DEBUG
                  break;
                }
                continue;
              }
#ifdef DEBUG
              _TRACELIST("%s Bad signal QSTATE=4\n", __func__);
#endif //DEBUG
              QSTATE = 0;
              break;
            }
          } //Q(4)

        //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
        //* State 5 - Wait for the signal to stabilize at negateive values               *
        //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
        case 5: {                                                     //At this point a crossing has been detected, the sample stream is inspected looking for another
            adc_v1 = getADCsample();                           //crossing and values are updated. When two sucessive samples are negative the FSM is advanced
            adc_t1 = time_us_32();                             //to status 6.
            uint32_t tstop = adc_t1 + 1000;

            while (true) {
              adc_v2 = getADCsample();
              adc_t2 = time_us_32();
              if (adc_v1 <= adc_zero && adc_v2 <= adc_zero) {
                QSTATE = 6;
                break;
              }

              if (adc_v1 >= adc_zero && adc_v2 <= adc_zero) {
                t1[0] = adc_t1;
                t2[0] = adc_t2;
                v1[0] = adc_v1;
                v2[0] = adc_v2;

                adc_v1 = adc_v2;
                adc_t1 = adc_t2;
                if (time_us_32() > tstop) {
                  QSTATE = 0;
#ifdef DEBUG
                  _TRACELIST("%s Break QSTATE=5\n", __func__);
#endif //DEBUG
                  break;
                }
                continue;
              }
              QSTATE = 0;
#ifdef DEBUG
              _TRACELIST("%s Bad signal QSTATE=5\n", __func__);
#endif //DEBUG
              break;
            }
          } //Q(5)
        //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
        //* State 6 - Two epoch available, compute the frequency                         *
        //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
        case 6: {                                                                    //Datum for two sucessive crossings has been collected at this point
            //then a more precise computation of the exact epoch of each crossing
            double m = ((v2[0] - v1[0]) * 1.0 / (t2[0] - t1[0]) * 1.0);          //is performed. The frequency is computed as the projection of the difference
            double t0s = t1[0] + uint32_t((adc_zero - v1[0]) * 1.0 / m);         //between two sucessive crossings projected to a full second.
            m = ((v2[1] - v1[1]) * 1.0 / (t2[1] - t1[1]) * 1.0);
            double t1s = t1[1] + uint32_t((adc_zero - v1[1]) * 1.0 / m);
            /*---------------------------------------------------------*
               This operates as a squelch by insuring that during the
               frequency measurement both positive and negative values
               went in excess of the calibrated zero level +/-10%
               and not computing the frequency if the signal was too
               low because it would create large errors if so
              ---------------------------------------------------------*/
            //if (adc_low == false || adc_high == false) {
            if (adc_high == false) {
#ifdef DEBUG
              _TRACELIST("%s Input signal too low to low(%s)/high(%s) compute ADC min=%d zero=%d max=%d (uh=%d/ul=%d) SAMPLE v(0)=%d/%d v(1)=%d/%d\n", __func__, BOOL2CHAR(adc_low), BOOL2CHAR(adc_high), adc_min, adc_zero, adc_max, adc_uh, adc_ul, v1[0], v2[0], v1[1], v2[1]);
#endif //DEBUG
              QSTATE = 7;
              break;
            }

            /*----------------------------------------------------------*
               If a valid epoch pair is detected then the frequency is
               computed, which is validated by allowing only baseband
               valid values to pass, this removes any high pitch reading
               because of measuring noise or a weak signal.
               Frequency is sent to the upcall caller working at core0
               and a rounding mechanism is applied (Dhiru Kholia's fix)
              ----------------------------------------------------------*/
            if ((t1s > t0s)) {
              double f = 1000000 / (t1s - t0s);
              if (f >= double(FSKMIN) && f <= double(FSKMAX)) {
                if (f < ffmin) {
                  ffmin = f;
                }
                if (f > ffmax) {
                  ffmax = f;
                }
                /*-----------------------------*
                   this is a valid f epoch
                  -----------------------------*/
                /*
                  fsequences[nfsi] = f;
                  rp2040.fifo.push_nb(nfsi);                  //Use the rp2040 FIFO IPC to communicate the new frequency
                  nfsi = (nfsi + 1) % NFS;
                  #ifdef DEBUG
                   _TRACELIST("%s dt=%ld dx=%.3f f=%.3f f1=%.3f ffsk=%ld\n",__func__,dt,dx,f,f1,ffsk);
                  #endif //DEBUG
                */
                ffsk = uint32_t(round(f));
                rp2040.fifo.push_nb(ffsk);                  //Use the rp2040 FIFO IPC to communicate the new frequency
#ifdef DEBUG
                _INFOLIST("%s f=%.2f Hz  fmin=%.2f fmax=%.2f  --> ffsk=%ld\n", __func__, f, ffmin, ffmax, ffsk);
#endif //DEBUG

              } else {
#ifdef DEBUG
                _TRACELIST("%s f=%.2f Hz  outside of frequency limits defined\n", __func__, f);
#endif //DEBUG
              }                                                   //

            } else {
#ifdef DEBUG
              _TRACELIST("%s Invalid frequency marking is ignored\n", __func__);
#endif //DEBUG
            }
            QSTATE = 7;

          } //Q(6)
        //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
        //* State 7 - Wait for the next measurement                                      *
        //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
        case 7: {                                                                     //In this state the FSM rest for a given delay
            sleep_ms(ADCSAMPLE);
            QSTATE = 0;
          }  //Q(7)

      }  //FSM(QSTATE) logic

    } //FSM(QSTATE) infinite loop
#endif //FSK_ADCZ
  }
}
/*==========================================================================================================*/

#ifdef EE
/*------------------------------------------------------------------------------*
   updateEEPROM
   selectively sets values into EEPROM
  ------------------------------------------------------------------------------*/
void updateEEPROM() {
  uint16_t save = EEPROM_SAVED;
  uint16_t build = BUILD;

  EEPROM.put(EEPROM_TEMP, save);
  EEPROM.put(EEPROM_BUILD, build);
  EEPROM.put(EEPROM_CAL, cal_factor);
  EEPROM.put(EEPROM_MODE, mode);
  EEPROM.put(EEPROM_BAND, Band_slot);

#ifdef TERMINAL

#ifdef ATUCTL
  EEPROM.put(EEPROM_ATU, atu);
  EEPROM.put(EEPROM_ATU_DELAY, atu_delay);
#endif //ATUCTL

  EEPROM.put(EEPROM_BOUNCE_TIME, bounce_time);
  EEPROM.put(EEPROM_SHORT_TIME, short_time);
  EEPROM.put(EEPROM_MAX_BLINK, max_blink);
  EEPROM.put(EEPROM_EEPROM_TOUT, eeprom_tout);


#endif //TERMINAL

  EEPROM.commit();
#ifdef DEBUG
  _INFOLIST("%s commit()\n", __func__)
#endif //DEBUG
  setWord(&SSW, SAVEEE, false);

#ifdef DEBUG
  _INFOLIST("%s save(%d) cal(%d) m(%d) slot(%d) save=%d build=%d\n", __func__, save, cal_factor, mode, Band_slot, save, build)
#endif //DEBUG
}

/*------------------------------------------------------------------------------*
   resetEEPROM
   reset to pinche defaults
  ------------------------------------------------------------------------------*/
void resetEEPROM() {
  uint16_t save = EEPROM_SAVED;
  uint16_t build = BUILD;

  mode = 0;
  Band_slot = 0;
  //* Retain calibration cal_factor=0;

#ifdef TERMINAL

#ifdef ATUCTL
  atu        = ATU;
  atu_delay  = ATU_DELAY;
#endif //ATUCTL

  bounce_time = BOUNCE_TIME;
  short_time = SHORT_TIME;
  max_blink  = MAX_BLINK;
  eeprom_tout = EEPROM_TOUT;

#endif //TERMINAL

  updateEEPROM();
}

/*------
   checkEEPROM
   check if there is a pending EEPROM save that needs to be committed
*/
void checkEEPROM() {
  if ((millis() - tout) > eeprom_tout && getWord(SSW, SAVEEE) == true ) {
#ifdef DEBUG
    _INFOLIST("%s() Saving EEPROM...\n", __func__);
#endif //DEBUG

    updateEEPROM();
  }
}

#endif //EE
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

  //@@@ freq = f[mode];  //Fix

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

  /*--------------------------------------------*
     Update master frequency here
    --------------------------------------------*/
  //@@@#ifdef EE
  //@@@  tout = millis();
  //@@@  setWord(&SSW, SAVEEE, true);
  //@@@#endif //EE

#ifdef DEBUG
  _INFOLIST("%s mode(%d) f(%ld)\n", __func__, mode, f[mode]);
#endif //DEBUG
}

/*------------------------------------------------------------------*
   Assign index in slot[x][] table based on the band
  ------------------------------------------------------------------*/
uint8_t band2Slot(uint16_t b) {
  uint8_t s = -1;    //@@@ To be traslated to ADX
  switch (b) {
    case  80 : {
        s = 0;
        break;
      }
    case  60 : {
        s = 1;
        break;
      }
    case  40 : {
        s = 2;
        break;
      }
    case  30 : {
        s = 3;
        break;
      }
    case  20 : {
        s = 4;
        break;
      }
    case  17 : {
        s = 5;
        break;
      }
    case  15 : {
        s = 6;
        break;
      }
    case  12 : {
        s = 7;
        break;
      }
    case  10 : {
        s = 8;
        break;
      }
  }
#ifdef DEBUG
  _INFOLIST("%s() band=%d slot=%d\n", __func__, b, s);
#endif //DEBUG

  return s;

}
/*----------------------------------------------------------*
   Frequency assign (band dependant)
   @@@ Deprecated, no longer used has to be removed form ADX
  ----------------------------------------------------------*/
/*
void Freq_assign() {
  uint16_t Band = Bands[Band_slot];
  uint8_t  b = band2Slot(Band);

  setStdFreq(b);    //@@@ To transfer to ADX

  //@@@  for (int i = 0; i < MAXMODE; i++) {
  //@@@    f[i] = slot[b][i];
  //@@@  #ifdef WDT
  //@@@    wdt_reset();    //Although quick don't allow loops to occur without a wdt_reset()
  //@@@  #endif //WDT

  freq = f[mode];      //Actual frequency is set depending on the selected mode

#ifdef QUAD

  int q = band2QUAD(Band);
  if (q != -1) {
    setQUAD(b);
  }
#ifdef DEBUG
  _INFOLIST("%s Band=%d slot=%d quad=%d f=%ld\n", __func__, Band, b, q, freq);
#endif
#endif //PA and LPF daughter board defined

#ifdef ATUCTL
  flipATU();
#endif //ATUCTL
#ifdef EE
  tout = millis();
  setWord(&SSW, SAVEEE, true);
#endif //EE
  switch_RXTX(LOW);
  setWord(&SSW, VOX, false);
  setWord(&SSW, TXON, false);
#ifdef WDT
  wdt_reset();
#endif

#ifdef DEBUG
  _INFOLIST("%s B(%d) b[%d] m[%d] slot[%d] f[0]=%ld f[1]=%ld f[2]=%ld f[3]=%ld f=%ld\n", __func__, Band, b, mode, Band_slot, f[0], f[1], f[2], f[3], freq);
#endif //DEBUG
}
*/
/*----------------------------------------------------------*
   Band assignment based on selected slot
   [@@@] just blink the leds since the actual frequency is
   going to be changed by calling Freq_assign()
  ----------------------------------------------------------*/
void Band_assign(bool l) {    //@@@ Change behaviour

  if (l == true) {
    resetLED();
    blinkLED(LED[3 - Band_slot]);
    delay(DELAY_WAIT);             //This delay should be changed
  }

  //@@@Freq_assign();
  //@@@Mode_assign();

#ifdef DEBUG
  _INFOLIST("%s mode(%d) slot(%d) f=%ld\n", __func__, mode, Band_slot, freq);
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
      tout = millis();
      setWord(&SSW, SAVEEE, true);
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

    //@@@#ifdef EE
    //@@@    EEPROM.put(EEPROM_MODE, mode);
    //@@@#endif //EEPROM

    //@@@ change from direct EEPROM write to delayed EEPROM write
#ifdef EE
    tout = millis();
    setWord(&SSW, SAVEEE, true);
#endif //EE
    //@@@

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
    tout = millis();
    setWord(&SSW, SAVEEE, true);
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
//*                   Configuration Terminal Function                                           *
//* This is an optional function allowing to modify operational parameters without recompiling  *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*

#ifdef TERMINAL
/*====================================================================================================*/
/*                                     Command Line Terminal                                          */
/*====================================================================================================*/
/*----------------------------------------------------------------------------------------------------*
   Simple Serial Command Interpreter
   Code excerpts taken from Mike Farr (arduino.cc)

  ---------------------------------------------------------------------------------------------------*/

bool getCommand(char * commandLine)
{
  static uint8_t charsRead = 0;                      //note: COMAND_BUFFER_LENGTH must be less than 255 chars long
  //read asynchronously until full command input

  /*------------------------------------------*
     Read till the serial buffer is exhausted
    ------------------------------------------*/
  while (Serial.available()) {
    char c = tolower(Serial.read());
    switch (c) {
      case CR:      //likely have full command in buffer now, commands are terminated by CR and/or LS
      case LF:
        commandLine[charsRead] = NULLCHAR;       //null terminate our command char array
        if (charsRead > 0)  {
          charsRead = 0;                           //charsRead is static, so have to reset
          Serial.println(commandLine);
          return true;
        }
        break;
      case BS:                                            // handle backspace in input: put a space in last char
        if (charsRead > 0) {                              //and adjust commandLine and charsRead
          commandLine[--charsRead] = NULLCHAR;
          sprintf(hi, "%c%c%c", BS, SPACE, BS);
          Serial.print(hi);
        }
        break;
      default:
        // c = tolower(c);
        if (charsRead < COMMAND_BUFFER_LENGTH) {
          commandLine[charsRead++] = c;
        }
        commandLine[charsRead] = NULLCHAR;     //just in case
        break;
    }
  }
  return false;
}

/*----------------------------------------------------------------------------------*
   readNumber
   Reads either a 8 or 16 bit number
  ----------------------------------------------------------------------------------*/
uint16_t readNumber () {
  char * numTextPtr = strtok(NULL, delimiters);         //K&R string.h  pg. 250
  return atoi(numTextPtr);                              //K&R string.h  pg. 251
}
/*----------------------------------------------------------------------------------*
   readWord
   Reads a string of characters
*/
char * readWord() {
  char * word = strtok(NULL, delimiters);               //K&R string.h  pg. 250
  return word;
}
/*----------------------------------------------------------------------------------*
   nullCommand
   Handle a command that hasn't been identified
*/
void nullCommand(char * ptrToCommandName) {
  sprintf(hi, "Command not found <%s>\r\n", ptrToCommandName);
  Serial.print(hi);
}

/*----------------------------------------------------------------------------------*
   Command processor
*/

/*---
   generic parameter update
*/
int updateWord(uint16_t *parm) {
  int v = readNumber();
  if (v == 0) {
    return (*parm);
  }
  (*parm) = v;
  return v;
}
/*
   ---
   save command
*/
void perform_saveToken () {
#ifdef EE
  updateEEPROM();
  Serial.println();
  Serial.print("EEPROM values saved\r\n>");
#endif //EE
  return;
}
/*---
   reset command
   all operational values are reset to default values and then saved on EEPROM
*/
void perform_resetToken () {

#ifdef EE
  resetEEPROM();
  Serial.println();
  Serial.print("EEPROM reset to default values\r\n>");
#endif //EE

  return;

}

#ifdef EE
/*---
   list command
   List EEPROM content
*/
void perform_listToken () {

  Serial.println();
  Serial.println("EEPROM list");
  int i = EEPROM_CAL;
  while (i < EEPROM_END) {
    sprintf(hi, "%05d -- ", i);
    Serial.print(hi);
    for (int j = 0; j < 10; j++) {
      uint8_t b = EEPROM.read(i + j);
      sprintf(hi, "%02x ", b);
      Serial.print(hi);
    }
    Serial.println();
    i = i + 10;
  }
  Serial.print(">");

  return;
}
#endif //EE
/*---
   quit command
*/
void perform_quitToken () {
  const char * msgQuit = "Exiting terminal mode";
  printMessage(msgQuit);
  delay(200);
  resetFunc();
  return 0;
}
/*---
   help command
   This is a spartan and limited yet efficient way to list all commands available.
   All commands are defined contiguosly as pointers to text, therefore a pointer is initialized
   with the first command in the list and all pointers are explored sequentially till a text with XXX
   (which must be placed at the end of the list as a marker) is found.
   However, the compiler for it's own superior reasons might alter the sequence of commands in memory
   and even put other things which are unrelated to them, therefore only strings starting with '*' and
   between 2 and 5 in size are eligible of being a command. The initial '*' is ignored from the listing and
   from the command parsing by taken the pointer to the string + 1.
*/
void perform_helpToken() {
  char * p = atuToken;

  while (strcmp(p, "XXX") != 0) {
#ifdef WDT
    wdt_reset();
#endif
    if (strlen(p) >= 2 && strlen(p) <= 5 && p[0] == '*') {
      sprintf(hi, "%s, ", p + 1);
      Serial.print(hi);
    }
    p = p + strlen(p) + 1;
  }
  Serial.print("\r\n>");

}
/*-----------------------------------------------------------------------------*
   printCommand
   print received command as a confirmation
  -----------------------------------------------------------------------------*/
void printCommand(char * token, uint16_t rc) {

  sprintf(hi, "%s(%05d)\n\r>", token, rc);
  Serial.print(hi);

  return;
}
void printMessage(char * token) {
  sprintf(hi, "%s\n\r>", token);
  Serial.print(hi);
}
/*--------------------------------------------------*
   execCommand
   parse command and process recognized tokens return
   result (which is always numeric
  --------------------------------------------------*/
void execCommand(char * commandLine) {
  //  int result;

  char * ptrToCommandName = strtok(commandLine, delimiters);
  const char * msgSave = "Parameters saved";
  const char * msgReset = "Reset to default values";

#ifdef ATUCTL
  if (strcmp(ptrToCommandName, atuToken + 1)         == 0) {
    printCommand(ptrToCommandName, updateWord(&atu));
    return;
  }
  if (strcmp(ptrToCommandName, atu_delayToken + 1)   == 0) {
    printCommand(ptrToCommandName, updateWord(&atu_delay));
    return;
  }
#endif //ATUCTL

  if (strcmp(ptrToCommandName, bounce_timeToken + 1) == 0) {
    printCommand(ptrToCommandName, updateWord(&bounce_time));
    return;
  }
  if (strcmp(ptrToCommandName, short_timeToken + 1)  == 0) {
    printCommand(ptrToCommandName, updateWord(&short_time));
    return;
  }
  if (strcmp(ptrToCommandName, max_blinkToken + 1)   == 0) {
    printCommand(ptrToCommandName, updateWord(&max_blink));
    return;
  }

#ifdef EE
  if (strcmp(ptrToCommandName, eeprom_toutToken + 1) == 0) {
    printCommand(ptrToCommandName, updateWord(&eeprom_tout));
    return;
  }
  if (strcmp(ptrToCommandName, eeprom_listToken + 1) == 0) {
    perform_listToken();
    return;
  }
  if (strcmp(ptrToCommandName, resetToken + 1)       == 0) {
    perform_resetToken();
    printMessage(msgReset);
    return;
  }
#endif //EE

  if (strcmp(ptrToCommandName, saveToken + 1)        == 0) {
    perform_saveToken();
    printMessage(msgSave);
    return;
  }
  if (strcmp(ptrToCommandName, quitToken + 1)        == 0) {
    perform_quitToken();
    return;
  }
  if (strcmp(ptrToCommandName, helpToken + 1)        == 0) {
    perform_helpToken();
    return;
  }

  nullCommand(ptrToCommandName);
  return;
}
/*-----------------------------------------------------------------------------*
   execTerminal
   executes the terminal processor if enabled                                  *
  -----------------------------------------------------------------------------*/
void execTerminal() {

  sprintf(hi, "\n\rADX %s build(%03d) command interpreter\n\r", VERSION, uint16_t(BUILD));
  Serial.print(hi);

  uint8_t n = 3;
  while (n > 0) {
    resetLED();
    delay(200);
    setLED(FT8, false);
    setLED(FT4, false);
    setLED(WSPR, false);
    setLED(JS8, false);
    delay(200);
    n--;
#ifdef WDT
    wdt_reset();
#endif //WDT
  }
  while (getGPIO(UP) == LOW) {
#ifdef WDT
    wdt_reset();
#endif //WDT
  }
  Serial.println("entering command mode, <quit> to finalize <help> for help\r\n>");
  switch_RXTX(LOW);

  while (true) {

    //*--- TERMINAL serial configuration

    if (getCommand(cmdLine)) {
      execCommand(cmdLine);
#ifdef WDT
      wdt_reset();
#endif //WDT
    } else {
#ifdef WDT
      wdt_reset();
#endif //WDT
    }
    checkEEPROM();

  }

}
#endif //TERMINAL

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

  uint16_t temp = 0;
  uint16_t save = EEPROM_SAVED;
  uint16_t build = 0;


  EEPROM.get(EEPROM_TEMP, temp);
  EEPROM.get(EEPROM_BUILD, build);

#ifdef DEBUG
  _INFOLIST("%s EEPROM retrieved temp(%d) & Build(%d) BUILD=%d\n", __func__, temp, build, uint16_t(BUILD));
#endif //DEBUG


#ifdef EEPROM_CLR
  temp = -1;
#endif //EEPROM_CLR


  if (build != uint16_t(BUILD)) {
    resetEEPROM();
#ifdef DEBUG
    _INFOLIST("%s EEPROM Reset Build<> cal(%ld) m(%d) slot(%d)\n", __func__, cal_factor, mode, Band_slot);
#endif //DEBUG

  }

  if (temp != save) {

    updateEEPROM();

#ifdef DEBUG
    _INFOLIST("%s EEPROM Reset cal(%ld) m(%d) slot(%d)\n", __func__, cal_factor, mode, Band_slot);
#endif //DEBUG

  } else {

    /*-----------------------------------------------*
       get configuration initialization from EEPROM  *
      ------------------------------------------------*/

    EEPROM.get(EEPROM_CAL, cal_factor);

    /*---- Kludge Fix
           to overcome wrong initial values, should not difficult calibration
    */
    if (cal_factor < -31000) {
      cal_factor = 0;
    }
    /* end of kludge */

    EEPROM.get(EEPROM_MODE, mode);
    EEPROM.get(EEPROM_BAND, Band_slot);

#ifdef TERMINAL

#ifdef ATUCTL
    EEPROM.get(EEPROM_ATU, atu);
    EEPROM.get(EEPROM_ATU_DELAY, atu_delay);
#endif //ATUCTL

    EEPROM.get(EEPROM_BOUNCE_TIME, bounce_time);
    EEPROM.get(EEPROM_SHORT_TIME, short_time);
    EEPROM.get(EEPROM_MAX_BLINK, max_blink);
    EEPROM.get(EEPROM_EEPROM_TOUT, eeprom_tout);

#endif //TERMINAL



#ifdef DEBUG
    _INFOLIST("%s EEPROM Read cal(%ld) m(%d) slot(%d)\n", __func__, cal_factor, mode, Band_slot);
#endif //DEBUG
  }
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
  gpio_init(LED_BUILTIN);
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
  gpio_set_dir(LED_BUILTIN, GPIO_OUT);
  gpio_set_dir(WSPR, GPIO_OUT);
  gpio_set_dir(JS8, GPIO_OUT);
  gpio_set_dir(FT4, GPIO_OUT);
  gpio_set_dir(FT8, GPIO_OUT);

  gpio_set_dir(FSK, GPIO_IN);

#ifdef ATUCTL
  gpio_init(uint8_t(atu));
  gpio_set_dir (uint8_t(atu), GPIO_OUT);
  flipATU();
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
  Serial.begin(BAUD);
  Serial1.setTX(12);
  Serial1.setRX(13);
  Serial1.begin(BAUD);

#ifdef DEBUG
  const char * proc = "RP2040";
  _INFOLIST("%s: ADX Firmware V(%s) build(%d) board(%s)\n", __func__, VERSION, BUILD, proc);
#endif //DEBUG

#ifdef EE
  EEPROM.begin(512);
#ifdef DEBUG
  _INFOLIST("%s: EEPROM reserved (%d)\n", __func__, EEPROM.length());
#endif //DEBUG
#endif //EE
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
  _INFOLIST("%s ONE BAND feature activated\n", __func__);
#else
  _INFOLIST("%s MULTI BAND feature activated\n", __func__);
#endif //ONEBAND

#ifdef QUAD
  _INFOLIST("%s Quad Band filter support activated\n", __func__);
#endif //ONEBAND

#ifdef FSK_PEG
  _INFOLIST("%s PEG decoding algorithm used Mult(%d) Window[uSec]=%d \n", __func__, uint16_t(FSK_MULT), uint16_t(FSK_WINDOW_USEC));
#endif //ONEBAND

#ifdef FSK_ZCD
  _INFOLIST("%s ZCD decoding algorithm used\n", __func__);
#endif //ONEBAND

#endif //DEBUG

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

#ifdef DEBUG
  _INFOLIST("%s FSK detection algorithm started QFSK=%s QWAIT=%s ok\n", __func__, BOOL2CHAR(getWord(QSW, QFSK)), BOOL2CHAR(getWord(QSW, QWAIT)));
#endif //DEBUG

  delay(500);
  switch_RXTX(LOW);

#ifdef DEBUG
  _INFOLIST("%s switch_RXTX Low ok\n", __func__);
#endif //DEBUG

  //@@@ Mode_assign();

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
  if ((millis() - tATU) > atu_delay && getWord(TSW, ATUCLK) == true) {
    setWord(&TSW, ATUCLK, false);
    setGPIO(atu, LOW);
  }
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

  //=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
  //*                                                                                *
  //*                      PDX Counting Algorithm                                    *
  //*                                                                                *
  //=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
  /*---------------------------------------------------------------------------------*
     setup1 () is running on a different thread at core1 and sampling the frequency
     using either a pwm counting (FSK_PEG) or a pseudo zero crossing (FSK_ZCD) method
     Whenever the frequency falls within the [FSKMIN,FSKMAX] limits it's FIFOed here
     Additional heuristic of validation are also applied to manage counting common
     counting errors.
     FSK_PEG
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
     FSK_ZCD
     The counting algorithm has rounding errors in the range of <500 uSec because the
     error in the triggering level and the residual +/- 1 uSec counting error
     the frequency is filtered by the bandwidth level and also a rounding error of
     1 Hz, thus if the sampled frequency is off by +/- 1 Hz the difference is less
     than the change on the PSK tone and thus ignored
    ---------------------------------------------------------------------------------*/
  uint16_t n = VOX_MAXTRY;
  uint32_t qBad = 0;
  uint32_t qTot = 0;

  setWord(&SSW, VOX, false);

  while ( n > 0 ) {                                //Iterate up to 10 times looking for signal to transmit

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
      n = VOX_MAXTRY;
#endif //FSK_ZCD

#ifdef FSK_ADCZ
      codefreq = rp2040.fifo.pop();
      n = VOX_MAXTRY;
#endif //FSK_ADCZ

#ifdef DEBUG
      _INFOLIST("%s FIFO f=%ld\n", __func__, codefreq);
#endif //DEBUG

      /*------------------------------------------------------*
         Filter out frequencies outside the allowed bandwidth
        ------------------------------------------------------*/
      if (codefreq >= uint32_t(FSKMIN) && codefreq <= uint32_t(FSKMAX)) {
        n = VOX_MAXTRY;
        qTot++;

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
          n = VOX_MAXTRY;
          continue;
        }
        /*-----------------------------------------------------*
           If this is the first sample AFTER the one that set
           the VOX on then switch the frequency to it
          -----------------------------------------------------*/
#ifdef FSK_ZCD
        if (pfo != fo) {
          si5351.set_freq(((freq + fo) * 100ULL), SI5351_CLK0);
#ifdef DEBUG
          _INFOLIST("%s Frequency change fo=%ld\n", __func__, fo);
#endif //DEBUG
          pfo = fo;
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
    } else {
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
  }

  /*---------------------------------------------------------------------------------*
     when out of the loop no further TX activity is performed, therefore the TX is
     turned off and the board is set into RX mode
    ---------------------------------------------------------------------------------*/

  /*------------------------------*
     This is a development probe
     to measure the counting
     error obtained into the
     frequency checking
    ------------------------------*/
#ifdef DEBUG
#ifdef FSK_ZCD
  if (qTot != 0) {
    float r = 100.0 * (float(qBad * 1.0) / float(qTot * 1.0));
#ifdef DEBUG
    _INFOLIST("%s <eof> qBad=%ld qTot=%ld error=%.6f\n", __func__, qBad, qTot, r);
#endif //DEBUG
    qBad = 0;
    qTot = 0;
  }
#endif //FSK_ZCD
#endif //DEBUG
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
