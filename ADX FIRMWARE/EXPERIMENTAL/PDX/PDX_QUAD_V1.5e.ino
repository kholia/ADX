//*********************************************************************************************************
//********************* ADX - ARDUINO based DIGITAL MODES 4 BAND HF TRANSCEIVER ***************************

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
// 1 - Connect CAL test point and GND test point on ADX PCB to a Frequency meter or Scope that can measure 1 MHz up to 1 Hz accurately.
// 2 - Press SW2 / --->(CAL) pushbutton and hold.
// 4-  Power up with 12V or with 5V by using arduino USB socket while still pressing SW2 / --->(CAL) pushbutton.
// 5 - FT8 and WSPR LEDs will flash 3 times and stay lit. Now Release SW2 / --->(CAL). Now Calibration mode is active.
// 6 - Using SW1(<---) and SW2(--->) pushbuttons change the Calibration frequency.
// 7 - Try to read 1 Mhz = 1000000 Hz exact on  Frequency counter or Oscilloscope.
//     The waveform is Square wave so freqency calculation can be performed esaily.
// 7 - If you read as accurate as possible 1000000 Hz then calibration is done.
// 8 - Power off ADX.
//*******************************[ LIBRARIES ]*************************************************
/*-------------------------------------------------------------*
 * Define the runtime platform either PICO (Raspberry Pi Pico) *
 * or !PICO (Arduino ATMega328p)                               *
 *-------------------------------------------------------------*/
// #define ADX              1   //This is the standard ADX Arduino based board
#define PDX            1   //Compile for Raspberry Pi Pico board


//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                            EXTERNAL LIBRARIES USED                                          *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*

void serialEvent();
void checkCAT();
unsigned char doingCAT = 0;
bool txCAT = false;        // turned on if the transmitting due to a CAT command
char inTx = 0;                // it is set to 1 if in transmit mode (whatever the reason : cw, ptt or cat)
char isUSB = 0;

#include <Arduino.h>
#include <stdint.h>
#include <si5351.h>
#include "Wire.h"
#include <EEPROM.h>


#ifdef ADX
   #include <avr/wdt.h>
#endif //ADX

#ifdef PDX
   #include "pico/stdlib.h"
   #include "pico/binary_info.h"
   #include "hardware/gpio.h"
   #include "hardware/sync.h"
   #include "hardware/structs/ioqspi.h"
   #include "hardware/structs/sio.h"
   #include <stdio.h>
   #include "hardware/watchdog.h"
#endif //PDX
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                            VERSION HEADER                                                   *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
#define VERSION        "1.5e"
#define BUILD          131

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                            MACRO DEFINES                                                    *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
#define BOOL2CHAR(x)  (x==true ? "True" : "False")
#undef  _NOP
#define _NOP          (byte)0

#ifdef ADX
   void(* resetFunc) (void) = 0;  // declare reset fuction at address 0 //resetFunc(); to reboot
   #define getGPIO(x) digitalRead(x)
   #define setGPIO(x,y) digitalWrite(x,y)
#endif //ADX

#ifdef PDX
   #define resetFunc() while(true) {}
   #define getGPIO(x) gpio_get(x)
   #define setGPIO(x,y) gpio_put(x,y)
   #define PICODISPLAY 1
   #define wdt_reset() watchdog_update()
#endif //PDX
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                            FEATURE CONFIGURATION PROPERTIES                                 *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*

#ifdef ADX
   #define WDT            1      //Hardware and TX watchdog enabled
   #define EE             1      //User EEPROM for persistence
   #define CAT            1      //Enable CAT protocol over serial port
   #define ATUCTL         1      //Control external ATU device
   #define RESET          1      //Allow a board reset (*)-><Band Select> -> Press & hold TX button for more than 2 secs will reset the board (EEPROM preserved)
   #define ANTIVOX        1      //Anti-VOX enabled, VOX system won't operate for AVOXTIME mSecs after the TX has been shut down by the CAT system
   #define ONEBAND        1      //Forces a single band operation in order not to mess up because of a wrong final filter
/*
 * The following definitions are disabled but can be enabled selectively
 */
//#define CAL_RESET    1      //If enabled reset cal_factor when performing a new calibration()
//#define DEBUG        1      //DEBUG turns on different debug, information and trace capabilities, it is nullified when CAT is enabled to avoid conflicts
//#define TERMINAL     1      //Serial configuration terminal
//#define IC746        1      //CAT Protocol is ICOM 746
//#define CW           1      //Enable CW operation
//#define SHIFTLIMIT   1      //Enforces tunning shift range into +/- 15 KHz when in CW mode
//#define CAT_FULL     1      //Extend CAT support to the entire CAT command set (valid only for TS480)
//#define LEGACY       1      //Enable code AS IS version 1.1 for testing purposes

#endif //PICO

#ifdef PDX
   // #define WDT            1      //Hardware and TX watchdog enabled
   #define CAT            1      //Enable CAT protocol over serial port

    /* No special configuration options for the PICO yet */
#endif //PDX
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                      GENERAL PURPOSE GLOBAL DEFINITIONS                                     *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
#define BOUNCE_TIME    200           //mSec minimum to debounce
#define SHORT_TIME     10*BOUNCE_TIME //mSec minimum to consider long push
#define SI5351_REF     25000000UL   //change this to the frequency of the crystal on your si5351’s PCB, usually 25 or 27 MHz
#define CPU_CLOCK      16000000UL   //Processor clock
#define VOX_MAXTRY     10           //Max number of attempts to detect an audio incoming signal
#define CNT_MAX        65000        //Max count of timer1
#define FRQ_MAX        30000        //Max divisor for frequency allowed
#define BDLY           200          //Delay when blinking LED
#define DELAY_WAIT  BDLY*2       //Double Delay
#define DELAY_CAL   DELAY_WAIT/10
#define MAXMODE        5            //Max number of digital modes
#define MAX_BLINK      4            //Max number of blinks
#define BANDS          4            //Max number of bands allowed
#define MAXBAND       10            //Max number of bands defined (actually uses BANDS out of MAXBAND)
#define XT_CAL_F      33000         //Si5351 Calibration constant
#define CAL_STEP      500           //Calibration factor step up/down while in calibration (sweet spot experimentally found by Barb)
#define REPEAT_KEY    30            //Key repetition period while in calibration
#define WAIT          true          //Debouncing constant
#define NOWAIT        false         //Debouncing constant
#define SERIAL_TOUT   10
#define SERIAL_WAIT   2

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                      PIN ASSIGNMENTS                                                        *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
#ifdef ADX
   #define UP             2           //UP Switch
   #define DOWN           3           //DOWN Switch
   #define TXSW           4           //TX Switch

   #define AIN0           6           //(PD6)
   #define AIN1           7           //(PD7)

   #define RX             8           //RX Switch
   #define TX            13           //(PB5) TX LED

   #define WSPR           9           //WSPR LED
   #define JS8           10           //JS8 LED
   #define FT4           11           //FT4 LED
   #define FT8           12           //FT8 LED
#ifdef ATUCTL
   #define ATU            5       //ATU Device control line (flipped HIGH during 200 mSecs at a band change)
#endif //ATUCTL

#endif //ADX


#ifdef PDX

   #define AIN0            0      //Reserved To implement zero crossing detection algorithm
   #define AIN1            1      //Reserved To implement zero crossing detection algorithm
   #define RX             18      //RX Switch
   #define WSPR            9      //WSPR LED
   #define JS8            10      //JS8 LED
   #define FT4            11      //FT4 LED
   #define FT8             8      //FT8 LED
   #define TX             25      //TX LED  //should be changed to 22 once the initial debugging is completed

#ifdef PICODISPLAY
   #define UP             15      //UP Switch Mapped to GPIO15 which is Y on the PicoDisplay Board
   #define DOWN           13      //DOWN Switch Mapped to GPIO13 which is A on the PicoDisplay Board
   #define TXSW           14      //TX Switch Mapped to GPIO14 which is X on the PicoDisplay Board
#else
   #define UP             19      //UP Switch (this must be set to GPIO19 when running on a PDX board)
   #define DOWN           20      //DOWN Switch (this must be set to GPIO20 when running on a PDX board)
   #define TXSW           14      //TX Switch
#endif //PICODISPLAY


#ifdef ATUCTL
   #define ATU            21     //ATU Device control line (flipped HIGH during 200 mSecs at a band change)
#endif //ATUCTL
#endif //PDX

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                      GLOBAL STATE VARIABLE DEFINITIONS                                      *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
/*----------------------------------------------------------------*
 *  Global State Variables (Binary)                               *
 *----------------------------------------------------------------*/
#define TXON   0B00000001    //State of the TX
#define VOX    0B00000010    //Audio input detected
#define UPPUSH 0B00000100    //UP button pressed
#define DNPUSH 0B00001000    //DOWN button pressed
#define TXPUSH 0B00010000    //TXSW button pressed
#define CATTX  0B00100000    //TX turned on via CAT (disable VOX)
#define SAVEEE 0B01000000    //Mark of EEPROM updated
#define CWMODE 0B10000000    //CW Active
/*----------------------------------------------------------------*
 * Operating switch                                               *
 * ---------------------------------------------------------------*/
#define PUSHSTATE   0B00000001
#define SHORTPUSH   0B00000010    //simple push flag
#define LONGPUSH    0B00000100    //long push flag
#define INPROGRESS  0B00001000    //in progress mark
#define ATUCLK      0B00010000    //control the width of the ATU pulse
#define TX_WDT      0B00100000    //TX Watchdog has been activated
#define AVOX        0B01000000    //ANTI-VOX has been activated

/*----------------------------------------------------------------*
 * Miscellaneour definitions                                              *
 * ---------------------------------------------------------------*/
char hi[80];
#define BAUD            19200
#define INT0                0
#define INT1                1
#define INT2                2
#define RTIME            2000


//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                      CONSISTENCY RULES                                                      *
//* Feature definition might conflict among them so some consistency rules are applied to remove*
//* potential inconsistencies on the definitions                                                *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*


//*--- This is a transient rule, the porting to Pico is incomplete so no functions are allowed

#ifdef PDX
    #undef EE
    #undef QUAD
    #undef ATUCTL
    #undef RESET
    #undef ANTIVOX
    #undef ONEBAND

    #undef CAL_RESET
    #undef DEBUG
    #undef TERMINAL
    #undef IC746
    #undef CW
    #undef SHIFTLIMIT
    #undef CAT_FULL
    #undef LEGACY
#endif //PICO

//*--- If any usage of the serial port is made then serial constants needs to be set


//*--- If a QUAD multiband board is defined then the transceiver must not be a single band one

#if (defined(QUAD))
    #undef   ONEBAND
#endif //QUAD, no ONEBAND

//*--- If CAT is defined then DEBUG  can not be activated simultaneously

#if (defined(CAT) && defined(DEBUG))  //Rule for conflicting usage of the serial port
   #undef  DEBUG
#endif // CAT && DEBUG

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
 * Serial configuration terminal commands                   *
 *----------------------------------------------------------*/
#ifdef ATUCTL
const char *atuToken        = "*atu";
const char *atu_delayToken  = "*atd";
#endif //ATUCTL

const char *bounce_timeToken= "*bt";
const char *short_timeToken = "*st";
const char *max_blinkToken  = "*mbl";

#ifdef ANTIVOX
const char *avoxtime_Token  = "*vxt";
#endif //ANTIVOX

#ifdef EE
const char *eeprom_toutToken= "*eet";
const char *eeprom_listToken= "*list";
#endif //EE

#ifdef CW
const char *cw_shiftToken   = "*cws";
const char *cw_stepToken    = "*cwt";
#endif //CW


const char *saveToken       = "*save";
const char *quitToken       = "*quit";
const char *resetToken      = "*reset";
const char *helpToken       = "*help";
const char *endList         = "XXX";

#endif //TERMINAL

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*               DEBUG SUPPORT MACRO DEFINITIONS                                               *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*

/*****************************************************************
 * Trace and debugging macros (only enabled if DEBUG is set      *
 *****************************************************************/
//#define DEBUG 1

#ifdef DEBUG        //Remove comment on the following #define to enable the type of debug macro
   //#define INFO  1   //Enable _INFO and _INFOLIST statements
   #define EXCP  1   //Enable _EXCP and _EXCPLIST statements
   //#define TRACE 1   //Enable _TRACE and _TRACELIST statements
#endif //DEBUG
/*-------------------------------------------------------------------------*
 * Define Info,Exception and Trace macros (replaced by NOP if not enabled) *
 *-------------------------------------------------------------------------*/
#ifdef DEBUG
   #define _DEBUG           sprintf(hi,"%s: Ok\n",__func__); Serial.print(hi);
   #define _DEBUGLIST(...)  sprintf(hi,__VA_ARGS__);Serial.print(hi);
   #define print2(x,y) (Serial.print(x), Serial.println(y))

#else
   #define _DEBUG _NOP
   #define _DEBUGLIST(...)  _DEBUG
   #define print2(x,y) _DEBUG
#endif

#ifdef TRACE
   #define _TRACE           sprintf(hi,"%s: Ok\n",__func__); Serial.print(hi);
   #define _TRACELIST(...)  sprintf(hi,__VA_ARGS__);Serial.print(hi);
#else
   #define _TRACE _NOP
   #define _TRACELIST(...)  _TRACE
#endif

#ifdef INFO
   #define _INFO           sprintf(hi,"%s: Ok\n",__func__); Serial.print(hi);
   #define _INFOLIST(...)  sprintf(hi,__VA_ARGS__);Serial.print(hi);
#else
   #define _INFO _NOP
   #define _INFOLIST(...)  _INFO
#endif

#ifdef EXCP
   #define _EXCP           sprintf(hi,"%s: Ok\n",__func__); Serial.print(hi);
   #define _EXCPLIST(...)  sprintf(hi,__VA_ARGS__);Serial.print(hi);
#else
   #define _EXCP           _NOP
   #define _EXCPLIST(...)  _EXCP
#endif

#ifdef ATUCTL
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*               ATU RESET FUNCTION SUPPORT                                                    *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
   #define ATU_DELAY    200       //How long the ATU control line (D5) is held HIGH on band changes, in mSecs

   uint16_t atu       =  ATU;
   uint16_t atu_delay =  ATU_DELAY;
   uint32_t tATU=0;

#endif //ATUCTL

#ifdef ANTIVOX
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*       ANTIVOX FEATURE IF PTT IS CONTROLLED BY CAT AVOID NOISE TO PTT THE ADX                *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
   #define AVOXTIME    2000
   uint16_t avoxtime =   AVOXTIME;
   uint32_t tavox    =   0;
#endif //ANTIVOX

#ifdef EE
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*       DEFINITIONS RELATED TO THE USAGE OF EEPROM AS A PERMANENT STORAGE                     *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
   #define EEPROM_CAL          10
   #define EEPROM_BUILD        20
   #define EEPROM_TEMP         30
   #define EEPROM_MODE         40
   #define EEPROM_BAND         50

   #ifdef TERMINAL
      #define EEPROM_ATU          60
      #define EEPROM_ATU_DELAY    70
      #define EEPROM_BOUNCE_TIME  80
      #define EEPROM_SHORT_TIME   90
      #define EEPROM_MAX_BLINK   120
      #define EEPROM_EEPROM_TOUT 130
      #define EEPROM_CW_SHIFT    140
      #define EEPROM_CW_STEP     150
      #define EEPROM_AVOXTIME    170
      #define EEPROM_END         200
   #endif //TERMINAL

   uint32_t tout=0;

   //#define EEPROM_CLR     1   //Initialize EEPROM (only to be used to initialize contents)
   #define EEPROM_SAVED 100     //Signature of EEPROM being updated at least once
   #define EEPROM_TOUT  500     //Timeout in mSecs to wait till commit to EEPROM any change
#endif //EEPROM

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*       DEFINITIONS RELATED TO THE IMPLEMENTATION OF CW MODE                                  *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*

#ifdef CW
   #define CWSHIFT        600
   #define CWSTEP         500
   #define MAXSHIFT     15000
   #define CWSLOT           5

   uint16_t cwshift = CWSHIFT;
   uint16_t cwstep  = CWSTEP;
   uint16_t maxshift= MAXSHIFT;
   uint16_t lastmode=       0;

#endif //CW


//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*       GLOBAL VARIABLE DEFINITION                                                            *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
uint16_t bounce_time    = BOUNCE_TIME;
uint16_t short_time     = SHORT_TIME;
uint16_t vox_maxtry     = VOX_MAXTRY;
int      cnt_max        = CNT_MAX;
uint16_t max_blink      = MAX_BLINK;
uint8_t  SSW            = 0;          //System SSW variable (to be used with getWord/setWord)
uint16_t mode           = 0;          //Default to mode=0 (FT8)
uint16_t Band_slot      = 0;          //Default to Bands[0]=40
int32_t  cal_factor     = 0;
unsigned long Cal_freq  = 1000000UL; // Calibration Frequency: 1 Mhz = 1000000 Hz

unsigned long f[MAXMODE]                   = { 7074000, 7047500, 7078000, 7038600, 7030000};   //Default frequency assignment
const unsigned long slot[MAXBAND][MAXMODE] ={{ 3573000, 3575000, 3578000, 3568600, 3560000},   //80m [0]
                                             { 5357000, 5357000, 5357000, 5287200, 5346500},   //60m [1]
                                             { 7074000, 7047500, 7078000, 7038600, 7030000},   //40m [2]
                                             {10136000,10140000,10130000,10138700,10106000},   //30m [3]
                                             {14074000,14080000,14078000,14095600,14060000},   //20m [4]
                                             {18100000,18104000,18104000,18104600,18096000},   //17m [5]
                                             {21074000,21140000,21078000,21094600,21060000},   //15m [6]
                                             {28074000,28074000,28078000,28124600,28060000}};  //10m [7]


unsigned long freq      = f[mode];
const uint8_t LED[4]    = {FT8,FT4,JS8,WSPR};

/*-------------------------------------*
 * Manage button state                 *
 *-------------------------------------*/
uint8_t       button[3]   ={0,0,0};
unsigned long downTimer[3]={PUSHSTATE,PUSHSTATE,PUSHSTATE};



//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*       GLOBAL VARIABLE DEFINITION CONDITIIONAL TO DIFFERENT OPTIONAL FUNCTIONS               *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*

#ifdef EE
uint16_t eeprom_tout = EEPROM_TOUT;
#endif //EE


#ifdef CW
unsigned long freqCW      = f[CWSLOT]; //default assignment consistent with digital mode's default, 40m
#endif //CW

#if (defined(ATUCTL) || defined(WDT))
uint8_t  TSW=0;               //System timer variable (to be used with getWord/setWord);
#endif //Either ATU or WDT has been defined

#ifdef WDT
#define       WDT_MAX     130000
uint32_t      wdt_tout    = 0;
#endif //WDT

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                             BAND SELECT CONFIGURATION                                       *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
/*
 ADX can support up to 4 bands on board. Those 4 bands needs to be assigned to Band1 ... Band4 from supported 6 bands.
 To change bands press SW1 and SW2 simultaneously. Band LED will flash 3 times briefly and stay lit for the stored band. also TX LED will be lit to indicate
 that Band select mode is active. Now change band bank by pressing SW1(<---) or SW2(--->). When desired band bank is selected press TX button briefly to exit band select mode.
 Now the new selected band bank will flash 3 times and then stored mode LED will be lit.
 TX won't activate when changing bands so don't worry on pressing TX button when changing bands in band mode.
 Assign your prefered bands to B1,B2,B3 and B4
 Supported Bands are: 80m, 40m, 30m, 20m,17m, 15m
*/


#ifdef ONEBAND                                      //If defined selects a single band to avoid mistakes with PA filter
   const uint16_t Bands[BANDS] ={10,10,10,10};             //All bands the same (change to suit needs)
#else
   const uint16_t Bands[BANDS] ={40,30,20,10};             //Band1,Band2,Band3,Band4 (initial setup)
#endif //ONEBAND

/*-----------------------------------------------------------------------------------------------------*
 * This is the definition of the QUAD filter board switching, this board carries 4 filters and can
 * decode up to 4 bands, so by wiring each of the 4 selectors enable any 4 band group, in the standard
 * configuration bands are encoded as:
 *             80 -- 0 --  1
 *             60 -- 1 --  2
 *             40 -- 2 --  4
 *             30 -- 3 --  8
 *             20 -- 4 -- 16
 *             17 -- 5 -- 32
 *             15 -- 6 -- 64
 *             10 -- 7 --128
 * This position matches the position in the slot[][] array and no change is needed, a future expansion
 * will allow for all HF bands + 6 meters to be coded but the QUAD board will still be able to decode
 * only 8 positions so an indirection can be made. Meanwhile it's better not to touch the quads[] defs
 *
 */
#ifdef QUAD
#define QUADMAX         8
  const uint16_t quads[QUADMAX] = {80,60,40,30,20,17,15,10};
#endif //QUAD

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                    CODE INFRASTRUCTURE                                                      *
//* General purpose procedures and functions needed to implement services through the code      *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
#ifdef PDX
/*----------------------------------------------------------------------*
 * This is specific code for the RP2040 to manage the BOOTSEL button    *
 * this code is for debugging purposes only                             *
 *----------------------------------------------------------------------*/
 bool __no_inline_not_in_flash_func(get_bootsel_button)() {
    const uint CS_PIN_INDEX = 1;

/*--------------------------------------------------------------------------*
 *
 * Must disable interrupts, as interrupt handlers may be in flash, and we
 * are about to temporarily disable flash access!
 *--------------------------------------------------------------------------*/
    uint32_t flags = save_and_disable_interrupts();

/*---
 *
 *Set chip select to Hi-Z
 *
 *----*/
    hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,GPIO_OVERRIDE_LOW << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

/*------
  Note we can't call into any sleep functions in flash right now
*-------*/
    for (volatile int i = 0; i < 1000; ++i);

/*-----------
  The HI GPIO registers in SIO can observe and control the 6 QSPI pins.
  Note the button pulls the pin *low* when pressed.
*------------*/
    bool button_state = !(sio_hw->gpio_hi_in & (1u << CS_PIN_INDEX));

/*-----------
 * Need to restore the state of chip select, else we are going to have a
 * bad time when we return to code in flash!
 *-----------*/

    hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,GPIO_OVERRIDE_NORMAL << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);
    restore_interrupts(flags);
    return button_state;
}
#endif //PDX

/*-------------------------------------------*
 * getWord                                   *
 * get boolean bitwise pseudo-variable       *
 *-------------------------------------------*/
bool getWord (uint8_t SysWord, uint8_t v) {
  return SysWord & v;
}
/*-------------------------------------------*
 * setSSW                                    *
 * set boolean bitwise pseudo-variable       *
 *-------------------------------------------*/
void setWord(uint8_t* SysWord,uint8_t v, bool val) {
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

   setGPIO(atu,HIGH);
   setWord(&TSW,ATUCLK,true);
   tATU=millis();

   #ifdef DEBUG
      _TRACELIST("%s()\n",__func__);
   #endif //DEBUG
}
#endif //ATUCTL
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                   QUAD BOARD MANAGEMENT                                                     *
//*this is an optional function that support the configuration of the QUAD board on band changes*
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*

#ifdef QUAD
/*====================================================================================================*/
/*                                     QUAD Board management                                          */
/*====================================================================================================*/
/*-------------------------------------------------------------------*
 * band2QUAD                                                         *
 * Transform a band [80..10] into the QUAD number to activate a LPF  *
 *-------------------------------------------------------------------*/
 int band2QUAD(uint16_t b) {

  int q=-1;
  for (int i=0;i<QUADMAX;i++) {
    if (quads[i]==b) {
       q=i;
       break;
    }
  }
  #ifdef DEBUG
  _INFOLIST("%s band=%d quad=%d\n",__func__,b,q);
  #endif //DEBUG
  return q;
 }
/*-------------------------------------------------------------------*
 * setQUAD                                                           *
 * Set the QUAD filter with the proper slot [0..3]                   *
 *-------------------------------------------------------------------*/
void setQUAD(int LPFslot) {

   uint8_t s =0;
   s |= (1<<LPFslot);

   Wire.beginTransmission(0x20);   //I2C device address
   Wire.write(0x09);               // address port A
   Wire.write(s);                  // Band Relay value to write
   Wire.endTransmission();
   delay(100);

   #ifdef DEBUG
      _INFOLIST("%s() LPFslot=%d QUAD=%d\n",__func__,LPFslot,s);
   #endif //DEBUG

}
/*-------------------------------------------------------------------*
 * setupQUAD                                                          *
 * init the QUAD Board [0..3]                                        *
 *-------------------------------------------------------------------*/
void setupQUAD() {

   Wire.begin();                   // wake up I2C bus
   Wire.beginTransmission(0x20);   //I2C device address
   Wire.write(0x00);               // IODIRA register
   Wire.write(0x00);               // set entire PORT A as output
   Wire.endTransmission();

   #ifdef DEBUG
      _INFO;
   #endif //DEBUG

}
#endif //QUAD

/*---
 *  Forward reference prototypes
 *---*/

void switch_RXTX(bool t);     //advanced definition for compilation purposes (interface only)
void Mode_assign();           //advanced definition for compilation purposes
void Freq_assign();
void Band_assign();
uint16_t changeBand(uint16_t c);


/*---------------------------------------*
 * getBand                               *
 * get a band number from frequency      *
 * (-1) is unsupported band              *
 *---------------------------------------*/
int getBand(uint32_t f) {

   int b=-1;
   if (f>= 3500000 && f< 4000000) {b=80;}
   if (f>= 5350000 && f< 5367000) {b=60;}
   if (f>= 7000000 && f< 7300000) {b=40;}
   if (f>=10100000 && f<10150000) {b=30;}
   if (f>=14000000 && f<14350000) {b=20;}
   if (f>=18068000 && f<18168000) {b=17;}
   if (f>=21000000 && f<21450000) {b=15;}
   if (f>=28000000 && f<29700000) {b=10;}

#ifdef DEBUG
   _INFOLIST("%s() f=%ld band=%d\n",__func__,f,b);
#endif //DEBUG

   return b;
}
/*------------------------------------------------------------*
 * findSlot                                                   *
 * find the slot [0..3] on the Bands array (band slot)        *
 *------------------------------------------------------------*/
int findSlot(uint16_t band) {

  int s=-1;
  for (int i=0;i<BANDS;i++) {
    if (Bands[i]==band) {
       s=i;
       break;
    }
  }
#ifdef DEBUG
   _INFOLIST("%s() band=%d slot=%d\n",__func__,band,s);
#endif //DEBUG

  return s;

}
/*-------------------------------------------------------------*
 * setSlot                                                     *
 * set a slot consistent with the frequency, do not if not     *
 * supported                                                   *
 *-------------------------------------------------------------*/
int setSlot(uint32_t f) {

   int b=getBand(f);
   if (b == -1) {
       return Band_slot;
   }
   int s=findSlot(b);

#ifdef DEBUG
   _TRACELIST("%s() f=%ld band=%d slot=%d\n",__func__,f,band,s);
#endif //DEBUG

   return s;

}
/*-----------------------------------------------------------------*
 * getMode                                                         *
 * given the slot in the slot[][] array and the frequency returns  *
 * the mode that should be assigned, -1 if none can be identified  *
 *-----------------------------------------------------------------*/
int getMode(int s,uint32_t f) {
  int m=-1;
  for (int i=0;i<MAXMODE;i++) {
    if (slot[s][i]==f) {
       m=i;
       break;
    }
  }

  #ifdef DEBUG
  _INFOLIST("%s slot=%d f=%ld m=%d\n",__func__,s,f,m);
  #endif //DEBUG

  return m;
}

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                   SI5351 MANAGEMENT SUBSYSTEM                                               *
//* Most of the work is actually performed by the Si5351 Library used                           *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
Si5351 si5351;

/*--------------------------------------------------------------------------------------------*
 * Initialize DDS SI5351 object
 *--------------------------------------------------------------------------------------------*/

void setup_si5351() {
//------------------------------- SET SI5351 VFO -----------------------------------
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
  #endif //DEBUG

}
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                   LED MANAGEMENT SUBSYSTEM                                                  *
//* Functions to operate the 5 LED the ADX board has                                            *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*

/**********************************************************************************************/
/*                                      LED Management                                        */
/**********************************************************************************************/
void clearLED(uint8_t LEDpin) {
  setGPIO(LEDpin,LOW);
  #ifdef DEBUG
     _TRACELIST("%s pin=%d",__func__,LEDpin);
  #endif //DEBUG
}
/*-----
 * Turn off all LEDs
 */
void resetLED() {               //Turn-off all LEDs

   clearLED(WSPR);
   clearLED(JS8);
   clearLED(FT4);
   clearLED(FT8);

   #ifdef DEBUG
   _TRACE;
   #endif //DEBUG


}

/*-----
 * Set a particular LED ON
 */
void setLED(uint8_t LEDpin,bool clrLED) {      //Turn-on LED {pin}

   (clrLED==true ? resetLED() : void(_NOP));
   setGPIO(LEDpin,HIGH);

#ifdef DEBUG
   _INFOLIST("%s(%d)\n",__func__,LEDpin);
#endif //DEBUG

}

/*-------
 * Blink a given LED
 */
void blinkLED(uint8_t LEDpin) {    //Blink 3 times LED {pin}

#ifdef DEBUG
   _TRACELIST("%s (%d)\n",__func__,LEDpin);
#endif //DEBUG

   uint8_t n=(max_blink-1);

   while (n>0) {
       setGPIO(LEDpin,HIGH);
       delay(BDLY);
       setGPIO(LEDpin,LOW);
       delay(BDLY);
       n--;

       #ifdef WDT
          wdt_reset();
       #endif //WDT
   }
}

/*-----
 * LED on calibration mode
 */
void calibrateLED(){           //Set callibration mode

   setGPIO(WSPR, HIGH);
   setGPIO(FT8, HIGH);
   delay(DELAY_CAL);
}
/*-----
 * Signal band selection with LED   (THIS NEEDS TO BE REVIEWED TO ACTUALLY SHOW MORE THAN 4 BANDS
 */
void bandLED(uint16_t b) {         //b would be 0..3 for standard ADX or QUAD

  setLED(LED[3-b],true);

}


//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                   BUTTON MANAGEMENT SUBSYSTEM                                               *
//* Functions to operate the 3 push buttons the ADX board has                                   *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*

#ifdef ADX
/**********************************************************************************************/
/*                               PushButton Management                                        */
/**********************************************************************************************/
/*---
 * ISR Handler
 * Handle push button interrupt
 */

ISR (PCINT2_vect) {

  uint32_t timerDown=0;
  byte     v=0;

  for (byte p=INT0;p<=INT2;p++){

      #ifdef DEBUG
         _TRACELIST("%s check pin(%d)\n",__func__,p);
      #endif //DEBUG

      switch (p) {
        case INT0 : {v=UPPUSH;break;}
        case INT1 : {v=DNPUSH;break;}
        case INT2 : {v=TXPUSH;break;}
      }
      bool pstate=PIND & v;
      if (pstate != getWord(button[p],PUSHSTATE)) {   //Evaluate which pin changed

//*--- Change detected

         #ifdef DEBUG
            _TRACELIST("%s pin(%d) [%d]->[%d]\n",__func__,p,getWord(button[p],PUSHSTATE),pstate);
         #endif //DEBUG

         setWord(&button[p],PUSHSTATE,pstate);
         if (pstate == LOW) {
           downTimer[p]=millis();
         } else {
           timerDown=millis()-downTimer[p];
           if (timerDown<bounce_time) {
              #ifdef DEBUG
                 _TRACELIST("%s pin(%d) too short, ignored!\n",__func__,p);
              #endif //DEBUG
              downTimer[p]=millis();  //fix weird Barb pushbutton with a 2nd train of bouncing signals

           }
           setWord(&SSW,v,true);
           if (timerDown<short_time){
              setWord(&button[p],SHORTPUSH,true);
              setWord(&button[p],LONGPUSH,false);

              #ifdef DEBUG
                 _TRACELIST("%s pin(%d) <SP>\n",__func__,p);
              #endif //DEBUG

           } else {
              setWord(&button[p],SHORTPUSH,false);
              setWord(&button[p],LONGPUSH,true);
              #ifdef DEBUG
                 _TRACELIST("%s pin(%d) <LP>\n",__func__,p);
              #endif //DEBUG
           }
         }
      }
  }
}
#endif //ADX


/*-----------------------------------------------------------------------------*
 * detectKey                                                                   *
 * detect if a push button is pressed                                          *
 *-----------------------------------------------------------------------------*/
bool detectKey(uint8_t k, bool v, bool w) {

   uint32_t tdown=millis();
   if (getGPIO(k)==v) {


      while (millis()-tdown<REPEAT_KEY) {
#ifdef WDT
          wdt_reset();
#endif //WDT
      }
      if (getGPIO(k)==v) { //confirmed as v value now wait for the inverse, if not return the inverse

          if (w==false) {
             return v;
          }

          while (true) {
#ifdef WDT
              wdt_reset();
#endif //WDT

              if (getGPIO(k)!=v) {
                 tdown=millis();
                 while (millis()-tdown<REPEAT_KEY) {
#ifdef WDT
              wdt_reset();
#endif //WDT
                 }
                 if (getGPIO(k)!=v) {
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
 * Switch between RX and TX
 *---------------------------------------------------------------------------------------------*/
void switch_RXTX(bool t) {  //t=False (RX) : t=True (TX)

#ifdef DEBUG
  if (t != getWord(SSW,TXON)) {
      _INFOLIST("%s (%s)\n",__func__,BOOL2CHAR(t));
  }
#endif //DEBUG

  if (t) {    //Set to TX

/*-----------------------------------*
 * if Watchdog enabled avoid to turn *
 * TX on if the watchdog mark hasn't *
 * been cleared.                     *
 *-----------------------------------*/
 #ifdef WDT

      if (getWord(TSW,TX_WDT)==HIGH) {
         return;
      }

 #endif //WDT
/*-----------------------------------*
 *               TX                  *
 *-----------------------------------*/
     setGPIO(RX,LOW);
     si5351.output_enable(SI5351_CLK1, 0);   //RX off

#ifdef CW
     long int freqtx=(getWord(SSW,CWMODE)==false ? freq : freq+cwshift);
#else
     long int freqtx=freq;
#endif //CW

#ifdef DEBUG
     _INFOLIST("%s TX+ f=%ld\n",__func__,freqtx);
#endif //DEBUG

#ifdef PDX
     digitalWrite(LED_BUILTIN, HIGH);
#endif

     si5351.set_freq(freqtx*100ULL, SI5351_CLK0);
     si5351.output_enable(SI5351_CLK0, 1);   // TX on
     setGPIO(TX,HIGH);
     setWord(&SSW,TXON,HIGH);

#ifdef WDT
     wdt_tout=millis();
#endif //WDT

     return;
  }

/*------------------------------------*
 *                RX                  *
 *------------------------------------*/
#ifdef CAT
    if (getWord(SSW,CATTX)==true) { return;}  //if the PTT is managed by the CAT subsystem get out of the way.
#endif //CAT

    setGPIO(RX,HIGH);
    si5351.output_enable(SI5351_CLK0, 0);   //TX off

#ifdef DEBUG
    if (getWord(SSW,TXON)==HIGH) {
       _TRACELIST("%s RX+ f=%ld\n",__func__,freq);
    }
#endif //DEBUG

    si5351.set_freq(freq*100ULL, SI5351_CLK1);
    si5351.output_enable(SI5351_CLK1, 1);   //RX on
    setGPIO(TX,0);
    setWord(&SSW,TXON,LOW);
    setWord(&SSW,VOX,LOW);
#ifdef PDX
     digitalWrite(LED_BUILTIN, LOW);
#endif

/*---------------------------------------------------------*
 * set to master frequency                                 *
 *---------------------------------------------------------*/

}


/*----------------------------------------------------------*
 * Manually turn TX while pressed                           *
 *----------------------------------------------------------*/
bool getTXSW();  //prototype for forward reference
void ManualTX(){

    bool buttonTX=getTXSW();
    switch_RXTX(HIGH);
    while(buttonTX==LOW) {

       #ifdef WDT
          wdt_reset();

          if ((millis() > (wdt_tout+uint32_t(WDT_MAX))) && getWord(SSW,TXON) == HIGH) {
             switch_RXTX(LOW);
             setWord(&TSW,TX_WDT,HIGH);
             wdt_tout=millis();
             return;
          }

          #ifdef CAT
             serialEvent();
          #endif //CAT

       #endif //WDT
       buttonTX=getTXSW();

    }
    switch_RXTX(LOW);

    #ifdef ANTIVOX
      tavox=millis();
      setWord(&TSW,AVOX,true);
    #endif //ANTIVOX
}

/*---------------------------------------------------------------------*
 * getSwitchPL
 * Detect and clear the Long push condition on both UP/DOWN buttons
 *---------------------------------------------------------------------*/
bool getSwitchPL(uint8_t pin) {

#ifdef ADX
//*--- pin can be 2,3,4

    byte p=pin-2;
    byte v=0;
    switch(p) {
      case INT0: {v=UPPUSH;break;}
      case INT1: {v=DNPUSH;break;}
      case INT2: {v=TXPUSH;break;}
    }

    if (getWord(SSW,v) == true && getWord(button[p],LONGPUSH)==true) {

       #ifdef DEBUG
          _TRACELIST("%s (%d): <PL>\n",__func__,p);
       #endif //DEBUG

       setWord(&SSW,v,false);
       setWord(&button[p],LONGPUSH,false);
       return LOW;
    } else {
       return HIGH;
    }

#endif //ADX

#ifdef PDX   //No support for Press Long feature yet
    return HIGH;
#endif //PDX
}
/*----------------------------------------------------------*
 * get value for a digital pin and return after debouncing  *
 *----------------------------------------------------------*/
bool getSwitch(uint8_t pin) {

#ifdef ADX
//*--- pin can be 2,3,4

    byte p=pin-2;
    byte v=0;
    switch(p) {
      case INT0: {v=UPPUSH;break;}
      case INT1: {v=DNPUSH;break;}
      case INT2: {v=TXPUSH;break;}
    }

    if (getWord(SSW,v) == true && getWord(button[p],SHORTPUSH)==true) {

       #ifdef DEBUG
          _TRACELIST("%s (%d): <SP>\n",__func__,p);
       #endif //DEBUG

       setWord(&SSW,v,false);
       setWord(&button[p],SHORTPUSH,false);
       return LOW;
    } else {
       return HIGH;
    }
#endif //ADX

#ifdef PDX
    return detectKey(pin,LOW,WAIT);
#endif //PDX


}
/*----------------------------------------------------------*
 * read UP switch
 *----------------------------------------------------------*/
bool getUPSSW() {

    return getSwitch(UP);

}
/*----------------------------------------------------------*
 * read DOWN Switch
 *----------------------------------------------------------*/
bool getDOWNSSW() {

    return getSwitch(DOWN);

}
/*--------------------------------------------------------------*
 * getTXSW() -- read TXSW switch
 * This switch still required debouncing but might operate
 * over long pulsing periods because of the manual TX function
 * and CW operation. It doesn't require to distinguish between
 * short and long pulse though.
 *---------------------------------------------------------------*/
bool getTXSW() {


#ifdef ADX
    if ( getWord(button[INT2],PUSHSTATE)==LOW && (millis()-downTimer[INT2]>bounce_time) ) {
       return LOW;
    }
    return HIGH;
#endif //ADX

#ifdef PDX
    return detectKey(TXSW,LOW,false);
#endif //PDX


}


/*----------------------------------------------------------*
 * Calibration function
 *----------------------------------------------------------*/
void Calibration(){


  #ifdef DEBUG
     _INFO;
  #endif //DEBUG

  resetLED();
  uint8_t  n=4;

  switch_RXTX(LOW);

  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA); // Set for lower power for calibration
  si5351.set_clock_pwr(SI5351_CLK0, 0); // Enable the clock for calibration

  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA); // Set for lower power for calibration
  si5351.set_clock_pwr(SI5351_CLK1, 0); // Enable the clock for calibration

  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA); // Set for lower power for calibration
  si5351.set_clock_pwr(SI5351_CLK2, 1); // Enable the clock for calibration

  /*-------
   * Reset calibration & apply initial values
   */

#ifdef CAL_RESET
  cal_factor=0;

  #ifdef EE
     EEPROM.put(EEPROM_CAL, cal_factor);
  #endif //EEPROM

#endif //CAL_RESET

  si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
  si5351.set_freq(Cal_freq * 100ULL, SI5351_CLK2);



  while (n>0) {

     #ifdef WDT
        wdt_reset();
     #endif //WDT

     calibrateLED();
     n--;

     #ifdef WDT
        wdt_reset();
     #endif //WDT
  }

     #ifdef EE
        EEPROM.get(EEPROM_CAL,cal_factor);
     #endif //EEPROM

     while (getGPIO(DOWN)==LOW) {
         #ifdef WDT
         wdt_reset();
         #endif //WDT
     }

#ifdef DEBUG
     _INFOLIST("%s cal_factor=%ld\n",__func__,cal_factor);
#endif //DEBUG

  while (true) {

 bool upButton     = getUPSSW();
 bool downButton   = getDOWNSSW();

     #ifdef WDT
        wdt_reset();
     #endif //WDT

     if (detectKey(UP,LOW,NOWAIT)==LOW) {
        cal_factor = cal_factor - CAL_STEP;

        #ifdef EE
           EEPROM.put(EEPROM_CAL, cal_factor);
        #endif //EEPROM

        si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);

#ifdef DEBUG

        _INFOLIST("%s (-) cal_factor=%ld cal_freq=%ld\n",__func__,cal_factor,Cal_freq);

#endif //DEBUG

  // Set Calibration CLK output

        si5351.set_freq(Cal_freq * 100ULL, SI5351_CLK2);
        si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA); // Set for lower power for calibration
        si5351.set_clock_pwr(SI5351_CLK2, 1); // Enable the clock for calibration
     }


     if (detectKey(DOWN,LOW,NOWAIT)==LOW) {
        cal_factor = cal_factor + CAL_STEP;

        #ifdef EE
           EEPROM.put(EEPROM_CAL, cal_factor);
        #endif //EEPROM

        si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);

 // Set Calbration Clock output
#ifdef DEBUG

        _INFOLIST("%s (+) cal_factor=%ld cal_freq=%ld\n",__func__,cal_factor,Cal_freq);

#endif //DEBUG

        si5351.set_freq(Cal_freq * 100ULL, SI5351_CLK2);
        si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA); // Set for lower power for Calibration
        si5351.set_clock_pwr(SI5351_CLK2, 1); // Enable clock2

     }

  }
}

#ifdef EE
/*------------------------------------------------------------------------------*
 * updateEEPROM                                                                 *
 * selectively sets values into EEPROM                                          *
 *------------------------------------------------------------------------------*/
void updateEEPROM() {

uint16_t save=EEPROM_SAVED;
uint16_t build=BUILD;

   EEPROM.put(EEPROM_TEMP,save);
   EEPROM.put(EEPROM_BUILD,build);
   EEPROM.put(EEPROM_CAL,cal_factor);
   EEPROM.put(EEPROM_MODE,mode);
   EEPROM.put(EEPROM_BAND,Band_slot);

#ifdef TERMINAL

#ifdef ATUCTL
   EEPROM.put(EEPROM_ATU,atu);
   EEPROM.put(EEPROM_ATU_DELAY,atu_delay);
#endif //ATUCTL

   EEPROM.put(EEPROM_BOUNCE_TIME,bounce_time);
   EEPROM.put(EEPROM_SHORT_TIME,short_time);
   EEPROM.put(EEPROM_MAX_BLINK,max_blink);
   EEPROM.put(EEPROM_EEPROM_TOUT,eeprom_tout);

#ifdef ANTIVOX
   EEPROM.put(EEPROM_AVOXTIME,avoxtime);
#endif //ANTIVOX

#ifdef CW
   EEPROM.put(EEPROM_CW_SHIFT,cw_shift);
   EEPROM.put(EEPROM_CW_STEP,cw_step);
#endif //CW

#endif //TERMINAL

   setWord(&SSW,SAVEEE,false);

   #ifdef DEBUG
      _TRACELIST("%s save(%d) cal(%d) m(%d) slot(%d)\n",__func__,save,cal_factor,mode,Band_slot);
   #endif //DEBUG



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
   cal_factor=0;

#ifdef TERMINAL

#ifdef ATUCTL
   atu        = ATU;
   atu_delay  = ATU_DELAY;
#endif //ATUCTL

   bounce_time= BOUNCE_TIME;
   short_time = SORT_TIME;
   max_blink  = MAX_BLINK;
   eeprom_tout= EEPROM_TOUT;


#ifdef ANTIVOX
   avoxtime   = AVOXTIME;
#endif //ANTIVOX

#ifdef CW
   cw_shift   = CW_SHIFT;
   cw_step    = CW_STEP;
#endif //CW


#endif //TERMINAL

   updateEEPROM();
}
/*------
 * checkEEPROM
 * check if there is a pending EEPROM save that needs to be committed
 */
void checkEEPROM() {

    if((millis()-tout)>eeprom_tout && getWord(SSW,SAVEEE)==true ) {
       updateEEPROM();
    }

}

#endif //EE
/*********************************************************************************************/
/*                               Operational state management                                 */
/*********************************************************************************************/
/*----------------------------------------------------------*
 * Band increase                                            *
 *----------------------------------------------------------*/
uint16_t changeBand(uint16_t c) {
    uint16_t b=(Band_slot+c)%BANDS;
    #ifdef DEBUG
       _TRACELIST("%s() change=%d Band_slot=%d b=%d\n",__func__,c,Band_slot,b);
    #endif //DEBUG
    return b;
}
/*----------------------------------------------------------*
 * Mode assign                                              *
 *----------------------------------------------------------*/
#ifdef CW
void displayFrequencyCW();
#endif //CW

void Mode_assign(){

   freqf[mode];
   if (mode==4) {
      setWord(&SSW,CWMODE,true);

#ifdef CW
      freqCW=freq;
      displayFrequencyCW();
#endif //CW

   } else {
      setWord(&SSW,CWMODE,false);
      setLED(LED[mode],true);

   }

/*---------------------------------------*
 * Change the DDS frequency              *
 *---------------------------------------*/
    switch_RXTX(LOW);
    setWord(&SSW,VOX,false);
    setWord(&SSW,TXON,false);
#ifdef WDT
    wdt_reset();
#endif

/*--------------------------------------------*
 * Update master frequency here               *
 *--------------------------------------------*/

   #ifdef EE
      tout=millis();
      setWord(&SSW,SAVEEE,true);
   #endif //EE

   #ifdef DEBUG
      _INFOLIST("%s mode(%d) f(%ld)\n",__func__,mode,f[mode]);
   #endif //DEBUG
}

/*------------------------------------------------------------------*
 * Assign index in slot[x][] table based on the band                *
 *------------------------------------------------------------------*/
uint8_t band2Slot(uint16_t b) {

      uint8_t s=3;
      switch(b) {
         case  80 : {s=0;break;}
         case  60 : {s=1;break;}
         case  40 : {s=2;break;}
         case  30 : {s=3;break;}
         case  20 : {s=4;break;}
         case  17 : {s=5;break;}
         case  15 : {s=6;break;}
         case  10 : {s=7;break;}
      }
      #ifdef DEBUG
       _INFOLIST("%s() band=%d slot=%d\n",__func__,b,s);
      #endif //DEBUG

      return s;

}
/*----------------------------------------------------------*
 * Frequency assign (band dependant)                        *
 *----------------------------------------------------------*/
void Freq_assign(){

    uint16_t Band=Bands[Band_slot];
    uint8_t  b=band2Slot(Band);
    for (int i=0;i<MAXMODE;i++) {
      f[i]=slot[b][i];
      #ifdef WDT
         wdt_reset();    //Although quick don't allow loops to occur without a wdt_reset()
      #endif //WDT

    }
/*---------------------------------------*
 * Update master frequency here          *
 *---------------------------------------*/
    freq=f[mode];        //Actual frequency is set depending on the selected mode

#ifdef QUAD
/*---------------------------------------*
 * Update filter selection for QUAD      *
 *---------------------------------------*/
     int q=band2QUAD(Band);
     if (q!=-1) {
        setQUAD(b);
     }
     #ifdef DEBUG
        _INFOLIST("%s Band=%d slot=%d quad=%d f=%ld\n",__func__,Band,b,q,freq);
     #endif
#endif //PA and LPF daughter board defined

#ifdef ATUCTL
/*---------------------------------------*
 * Update ATU control                    *
 *---------------------------------------*/
     flipATU();

#endif //ATUCTL
/*---------------------------------------*
 * Flag EEPROM to be updated             *
 *---------------------------------------*/
    #ifdef EE
       tout=millis();
       setWord(&SSW,SAVEEE,true);
    #endif //EE

/*---------------------------------------*
 * Change the DDS frequency              *
 *---------------------------------------*/
    switch_RXTX(LOW);
    setWord(&SSW,VOX,false);
    setWord(&SSW,TXON,false);
#ifdef WDT
    wdt_reset();
#endif

    #ifdef DEBUG
       _INFOLIST("%s B(%d) b[%d] m[%d] slot[%d] f[0]=%ld f[1]=%ld f[2]=%ld f[3]=%ld f=%ld\n",__func__,Band,b,mode,Band_slot,f[0],f[1],f[2],f[3],freq);
    #endif //DEBUG
}

/*----------------------------------------------------------*
 * Band assignment based on selected slot                   *
 *----------------------------------------------------------*/
void Band_assign(){

    resetLED();
    blinkLED(LED[3-Band_slot]);

    delay(DELAY_WAIT);             //This delay should be changed

    Freq_assign();
    Mode_assign();

    #ifdef DEBUG
       _INFOLIST("%s mode(%d) slot(%d) f=%ld\n",__func__,mode,Band_slot,freq);
    #endif //DEBUG

}
/*----------------------------------------------------------*
 * Select band to operate
 *----------------------------------------------------------*/
void Band_Select(){

   resetLED();

   #ifdef DEBUG
      _INFOLIST("%s slot(%d) LED(%d)\n",__func__,Band_slot,LED[3-Band_slot]);
   #endif //DEBUG

   blinkLED(LED[3-Band_slot]);
   setLED(LED[3-Band_slot],true);
   bool     l=false;
   uint32_t t=millis();

   while (true) {
      #ifdef WDT
         wdt_reset();
      #endif //WDT

      if ((millis()-t) > uint32_t(BDLY)) {
         t=millis();
         (l==false ? setLED(TX,false) : clearLED(TX));
         l=!l;
      }

      bool upButton   = getUPSSW();
      bool downButton = getDOWNSSW();
      bool txButton   = getTXSW();

      if (detectKey(UP,LOW,WAIT)==LOW) {
          Band_slot=changeBand(-1);
          setLED(LED[3-Band_slot],true);

          #ifdef DEBUG
             _INFOLIST("%s slot(%d)\n",__func__,Band_slot);
          #endif //DEBUG
      }

      if (detectKey(DOWN,LOW,WAIT)==LOW) {
         Band_slot=changeBand(+1);
         setLED(LED[3-Band_slot],true);

         #ifdef DEBUG
            _INFOLIST("%s slot(%d)\n",__func__,Band_slot);
         #endif //DEBUG

      }
      if (txButton == LOW) {
         setGPIO(TX,LOW);

#ifdef RESET
         uint32_t tnow=millis();
         while (getTXSW()== LOW) {
            if (millis()-tnow>RTIME) {
                setLED(WSPR,false);
                setLED(JS8,false);
                setLED(FT4,false),
                setLED(FT8,false),
                delay(500);
                resetLED();
                delay(500);
            }
            #ifdef WDT
            wdt_reset();
            #endif //WDT
         }
         if (millis()-tnow > RTIME) {
         resetFunc();
       }
#endif //RESET

       #ifdef EE
         tout=millis();
         setWord(&SSW,SAVEEE,true);
       #endif //EE

       setLED(TX,false);
       Band_assign();
/*------------------------------------------*
 * Update master frequency                  *
 *------------------------------------------*/
       return;
   }
}
}
#ifdef CW
/*----------------------------------------------------------------*
 * select LED representation according with the frequency shift   *
 * with the center (initial) qrp calling frequency                *
 *----------------------------------------------------------------*/
void displayFrequencyCW() {

  if (freq==freqCW) {
     setLED(JS8,true);
     setLED(FT4,false);
     #ifdef DEBUG
        _TRACELIST("%s set QRP QRG f=%ld\n",__func__,freq);
     #endif //DEBUG
     return;
  }

  long int df=freq-freqCW;

/*-----------------------------------------------------------*
 * compute a fancy LED lighting to signal where the frequency*
 * is located compared with the QRP calling frequency (only  *
 * valid for CW mode when activated                          *
 *-----------------------------------------------------------*/
  if ((df>     0) && (df<= 5000)) {setLED(FT4,true);}
  if ((df>  5000) && (df<=10000)) {setLED(FT4,true);setLED(FT8,false);}
  if ((df> 10000)) {setLED(FT8,true);}
  if (df<0) {
     df=abs(df);
     if ((df>     0) && (df<= 5000)) {setLED(JS8,true);}
     if ((df>  5000) && (df<=10000)) {setLED(JS8,true);setLED(WSPR,false);}
     if ((df> 10000)) {setLED(WSPR,true);}
  }
}
#endif //CW

#ifdef CW
/*-------------------------------------------------------------*
 * setFrequencyCW tuning function when CW mode is active       *
 *-------------------------------------------------------------*/
void setFrequencyCW(int f) {

  long int step=f*cwstep;

  #ifdef DEBUG
     _TRACELIST("%s f=%ld\n",__func__,freq+step);
  #endif //DEBUG

#ifdef SHIFTLIMIT

  if ((freq+step)>(freqCW+maxshift)) {

     #ifdef DEBUG
        _TRACELIST("%s (%d): %ld out of band\n",__func__,step,freq+step);
     #endif //DEBUG

     blinkLED(FT8);
     setLED(FT8,true);
     return;
  }
  if ((freq+step)<(freqCW-maxshift)) {

     #ifdef DEBUG
        _TRACELIST("%s step=%ld f=%ld out of band\n",__func__,step,freq+step);
     #endif //DEBUG

     blinkLED(WSPR);
     setLED(WSPR,true);
     return;
  }
  #ifdef DEBUG
     _TRACELIST("%s f=%ld):\n",__func__,freq+step);
  #endif //DEBUG

#endif //SHIFTLIMIT

  freq=freq+step;
  displayFrequencyCW();
  return;

}
#endif //CW
/*---------------------------------------------------------------------------*
 *  checkMode
 *  manage change in mode during run-time
 *---------------------------------------------------------------------------*/
void checkMode() {


bool txButton     = getTXSW();
bool upButton     = getUPSSW();
bool downButton   = getDOWNSSW();
bool upButtonPL   = getSwitchPL(UP);
bool downButtonPL = getSwitchPL(DOWN);

/*------------------------------------------------------*
 * Manage change to and from CW mode by detecting a     *
 * long push press of the UP button (to enter CW) and a *
 * long push press of the DOWN button (to exit CW)      *
 *------------------------------------------------------*/
  #ifdef CW

/*--------------------------------*
 * UP button long press && !CW    *
 * Start CW mode                  *
 *--------------------------------*/

  if (upButtonPL == LOW && getWord(SSW,CWMODE)==false) {
     lastmode=mode;
     mode=4;
     Mode_assign();

     #ifdef DEBUG
        _INFOLIST("%s CW+ f=%ld\n",__func__,freq);
     #endif //DEBUG

  }

/*--------------------------------*
 * DN button long press && CW     *
 * Exit CW mode                   *
 *--------------------------------*/
  if (downButtonPL == LOW && getWord(SSW,CWMODE)== true) {
     mode=lastmode;
     Mode_assign();

     #ifdef DEBUG
        _INFOLIST("%s CW- f=%ld\n",__func__,freq);
     #endif //DEBUG

  }

/*-----------------------------------------------------------*
 * While in CW mode UP means frequency up by CWSTEP and DOWN *
 * means frequency down by CWSTEP Hz. The tunning range is   *
 * +/- MAXRIT                                                *
 *-----------------------------------------------------------*/
  if (downButton == LOW && getWord(SSW,CWMODE)==true) {
     setFrequencyCW(-1);

     #ifdef DEBUG
        _INFOLIST("%s f+ f=%ld\n",__func__,freq);
     #endif //DEBUG
  }

  if (upButton == LOW && getWord(SSW,CWMODE)==true) {
     setFrequencyCW(+1);

     #ifdef DEBUG
        _INFOLIST("%s f- f=%ld\n",__func__,freq);
     #endif //DEBUG
  }
//*-------------------------[CW Implementation]------------------

  #endif //CW

/*--------------------------------*
 * TX button short press          *
 * Transmit mode                  *
 *--------------------------------*/

    if ((txButton == LOW) && (getWord(SSW,TXON)==false)) {
       #ifdef DEBUG
          _EXCPLIST("%s TX+\n",__func__);
       #endif //DEBUG

       ManualTX();

       #ifdef DEBUG
         _EXCPLIST("%s TX-\n",__func__);
       #endif //DEBUG
  }

/*------------------------------------------------------------*
 * while in CW mode just block band and mode changes          *
 *------------------------------------------------------------*/

#ifdef CW

  if (getWord(SSW,CWMODE)==true) {
     return;
  }
#endif //CW

/*-------------------------------------------------------------*
 * manage band, mode and  calibration selections               *
 *-------------------------------------------------------------*/
#ifndef ONEBAND

  if ((upButton == LOW)&&(downButton == LOW)&&(getWord(SSW,TXON)==false)) {
     Band_Select();

     #ifdef DEBUG
      _EXCPLIST("%s U+D f=%ld",__func__,freq);
     #endif //DEBUG
  }
#endif //ONEBAND

  if ((upButton == HIGH)&&(downButton == LOW)&&(getWord(SSW,TXON)==false)) {

      if (mode==0) {
        mode=3;
      } else {
        mode--;
      }

      #ifdef DEBUG
         _EXCPLIST("%s m+(%d)\n",__func__,mode);
      #endif //DEBUG

      #ifdef EE
         EEPROM.put(EEPROM_MODE, mode);
      #endif //EEPROM

      Mode_assign();

      #ifdef DEBUG
         _EXCPLIST("%s mode assigned(%d)\n",__func__,mode);
      #endif //DEBUG

  }


  if ((upButton == LOW) && (downButton == HIGH)&&(getWord(SSW,TXON)==false)) {

      mode=(mode+1)%4;


      #ifdef EE
         tout=millis();
         setWord(&SSW,SAVEEE,true);
      #endif //EE Avoid the tear and wear of the EEPROM because of successive changes

      #ifdef DEBUG
         _EXCPLIST("%s m-(%d)\n",__func__,mode);
      #endif //DEBUG

      Mode_assign();

      #ifdef DEBUG
         _EXCPLIST("%s mode assigned(%d)\n",__func__,mode);
      #endif //DEBUG

  }



}
/*---------------------------------------------------------------------------------*
 * keepAlive()
 * Reference function for debugging purposes, called once per loop
 *---------------------------------------------------------------------------------*/
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
 * Simple Serial Command Interpreter
 * Code excerpts taken from Mike Farr (arduino.cc)
 *
 *---------------------------------------------------------------------------------------------------*/

bool getCommand(char * commandLine)
{
  static uint8_t charsRead = 0;                      //note: COMAND_BUFFER_LENGTH must be less than 255 chars long
  //read asynchronously until full command input

  /*------------------------------------------*
   * Read till the serial buffer is exhausted *
   *------------------------------------------*/
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
          sprintf(hi,"%c%c%c",BS,SPACE,BS);
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
 * readNumber                                                                       *
 * Reads either a 8 or 16 bit number                                                *
 *----------------------------------------------------------------------------------*/
uint16_t readNumber () {
  char * numTextPtr = strtok(NULL, delimiters);         //K&R string.h  pg. 250
  return atoi(numTextPtr);                              //K&R string.h  pg. 251
}
/*----------------------------------------------------------------------------------*
 * readWord
 * Reads a string of characters
 */
char * readWord() {
  char * word = strtok(NULL, delimiters);               //K&R string.h  pg. 250
  return word;
}
/*----------------------------------------------------------------------------------*
 * nullCommand
 * Handle a command that hasn't been identified
 */
void nullCommand(char * ptrToCommandName) {
  sprintf(hi,"Command not found <%s>\r\n",ptrToCommandName);
  Serial.print(hi);
  }

/*----------------------------------------------------------------------------------*
 * Command processor
 */

/*---
 * generic parameter update
 */
int updateWord(uint16_t *parm) {
    int v=readNumber();
    if (v==0) {
       return (*parm);
    }
    (*parm)=v;
    return v;
}
/*
 * ---
 * save command
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
 * reset command
 * all operational values are reset to default values and then saved on EEPROM
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
 * list command
 * List EEPROM content
 */
void perform_listToken () {

    Serial.println();
    Serial.println("EEPROM list");
    int i=EEPROM_CAL;
    while(i<EEPROM_END) {
      sprintf(hi,"%05d -- ",i);
      Serial.print(hi);
      for (int j=0;j<10;j++) {
        uint8_t b=EEPROM.read(i+j);
        sprintf(hi,"%02x ",b);
        Serial.print(hi);
      }
      Serial.println();
      i=i+10;
    }
    Serial.print(">");

    return;
}
#endif //EE
/*---
 * quit command
 */
void perform_quitToken () {
const char * msgQuit = "Exiting terminal mode";
    printMessage(msgQuit);
    delay(200);
    resetFunc();
    return 0;
}
/*---
 * help command
 * This is a spartan and limited yet efficient way to list all commands available.
 * All commands are defined contiguosly as pointers to text, therefore a pointer is initialized
 * with the first command in the list and all pointers are explored sequentially till a text with XXX
 * (which must be placed at the end of the list as a marker) is found.
 * However, the compiler for it's own superior reasons might alter the sequence of commands in memory
 * and even put other things which are unrelated to them, therefore only strings starting with '*' and
 * between 2 and 5 in size are eligible of being a command. The initial '*' is ignored from the listing and
 * from the command parsing by taken the pointer to the string + 1.
 */
void perform_helpToken(){
 char * p = atuToken;

 while (strcmp(p,"XXX")!=0) {
    #ifdef WDT
       wdt_reset();
    #endif
    if (strlen(p)>=2 && strlen(p)<=5 && p[0]=='*') {
       sprintf(hi,"%s, ",p+1);
       Serial.print(hi);
    }
    p=p+strlen(p)+1;
  }
  Serial.print("\r\n>");

}
/*-----------------------------------------------------------------------------*
 * printCommand                                                                *
 * print received command as a confirmation                                    *
 *-----------------------------------------------------------------------------*/
void printCommand(char * token, uint16_t rc) {

  sprintf(hi,"%s(%05d)\n\r>",token,rc);
  Serial.print(hi);

  return;
}
void printMessage(char * token) {
  sprintf(hi,"%s\n\r>",token);
  Serial.print(hi);
}
/*--------------------------------------------------*
   execCommand
   parse command and process recognized tokens return
   result (which is always numeric
 *--------------------------------------------------*/
void execCommand(char * commandLine) {
//  int result;

  char * ptrToCommandName = strtok(commandLine, delimiters);
  const char * msgSave = "Parameters saved";
  const char * msgReset= "Reset to default values";

#ifdef ATUCTL
  if (strcmp(ptrToCommandName, atuToken+1)         == 0) {printCommand(ptrToCommandName,updateWord(&atu));return;}
  if (strcmp(ptrToCommandName, atu_delayToken+1)   == 0) {printCommand(ptrToCommandName,updateWord(&atu_delay));return;}
#endif //ATUCTL

  if (strcmp(ptrToCommandName, bounce_timeToken+1) == 0) {printCommand(ptrToCommandName,updateWord(&bounce_time));return;}
  if (strcmp(ptrToCommandName, short_timeToken+1)  == 0) {printCommand(ptrToCommandName,updateWord(&short_time));return;}
  if (strcmp(ptrToCommandName, max_blinkToken+1)   == 0) {printCommand(ptrToCommandName,updateWord(&max_blink));return;}

#ifdef EE
  if (strcmp(ptrToCommandName, eeprom_toutToken+1) == 0) {printCommand(ptrToCommandName,updateWord(&eeprom_tout));return;}
  if (strcmp(ptrToCommandName, eeprom_listToken+1) == 0) {perform_listToken();return;}
  if (strcmp(ptrToCommandName, resetToken+1)       == 0) {perform_resetToken();printMessage(msgReset);return;}
#endif //EE

#ifdef ANTIVOX
  if (strcmp(ptrToCommandName, avoxtime_Token+1)   == 0) {printCommand(ptrToCommandName,updateWord(&avoxtime));return;}
#endif //ANTIVOX

#ifdef CW
  if (strcmp(ptrToCommandName, cw_shiftToken+1)    == 0) {printCommand(ptrToCommandName,updateWord(&cwshift));return;}
  if (strcmp(ptrToCommandName, cw_stepToken+1)     == 0) {printCommand(ptrToCommandName,updateWord(&cwstep));return;}
#endif //CW

  if (strcmp(ptrToCommandName, saveToken+1)        == 0) {perform_saveToken();printMessage(msgSave);return;}
  if (strcmp(ptrToCommandName, quitToken+1)        == 0) {perform_quitToken();return;}
  if (strcmp(ptrToCommandName, helpToken+1)        == 0) {perform_helpToken();return;}

  nullCommand(ptrToCommandName);
return;
}
/*-----------------------------------------------------------------------------*
 * execTerminal                                                                *
 * executes the terminal processor if enabled                                  *                                          *
 *-----------------------------------------------------------------------------*/
void execTerminal() {

   sprintf(hi,"\n\rADX %s build(%03d) command interpreter\n\r",VERSION,uint16_t(BUILD));
   Serial.print(hi);

   uint8_t n=3;
   while (n>0) {
      resetLED();
      delay(200);
      setLED(FT8,false);
      setLED(FT4,false);
      setLED(WSPR,false);
      setLED(JS8,false);
      delay(200);
      n--;
      #ifdef WDT
         wdt_reset();
      #endif //WDT
   }
   while (getGPIO(UP)==LOW) {
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
 * Initialization function from EEPROM                      *
 *----------------------------------------------------------*/
void initADX(){


#ifdef EE

 uint16_t temp=0;
 uint16_t save=EEPROM_SAVED;
 uint16_t build=0;


 EEPROM.get(EEPROM_TEMP,temp);
 EEPROM.get(EEPROM_BUILD,build);

 #ifdef EEPROM_CLR
    temp=-1;
 #endif //EEPROM_CLR


 if (build != uint16_t(BUILD)) {
    resetEEPROM();
    #ifdef DEBUG
       _INFOLIST("%s EEPROM Reset Build<> cal(%ld) m(%d) slot(%d)\n",__func__,cal_factor,mode,Band_slot);
    #endif //DEBUG

 }

 if (temp != save){

    updateEEPROM();

    #ifdef DEBUG
       _INFOLIST("%s EEPROM Reset cal(%ld) m(%d) slot(%d)\n",__func__,cal_factor,mode,Band_slot);
    #endif //DEBUG

 } else {

   /*-----------------------------------------------*
    * get configuration initialization from EEPROM  *            *
    ------------------------------------------------*/

  EEPROM.get(EEPROM_CAL,cal_factor);

/*---- Kludge Fix
 *     to overcome wrong initial values, should not difficult calibration
 */
  if (cal_factor < -31000) {
      cal_factor=0;
  }
/* end of kludge */

  EEPROM.get(EEPROM_MODE,mode);
  EEPROM.get(EEPROM_BAND,Band_slot);

#ifdef TERMINAL

#ifdef ATUCTL
   EEPROM.get(EEPROM_ATU,atu);
   EEPROM.get(EEPROM_ATU_DELAY,atu_delay);
#endif //ATUCTL

   EEPROM.get(EEPROM_BOUNCE_TIME,bounce_time);
   EEPROM.get(EEPROM_SHORT_TIME,short_time);
   EEPROM.get(EEPROM_MAX_BLINK,max_blink);
   EEPROM.get(EEPROM_EEPROM_TOUT,eeprom_tout);

#ifdef ANTIVOX
   EEPROM.get(EEPROM_AVOXTIME,avoxtime);
#endif //ANTIVOX

#ifdef CW
   EEPROM.get(EEPROM_CW_SHIFT,cw_shift);
   EEPROM.get(EEPROM_CW_STEP,cw_step);
#endif //CW


#endif //TERMINAL


  setup_si5351();

  #ifdef DEBUG
     _INFOLIST("%s EEPROM Read cal(%ld) m(%d) slot(%d)\n",__func__,cal_factor,mode,Band_slot);
  #endif //DEBUG
}

#endif // EE

  Band_assign();
  Freq_assign();
  Mode_assign();
  switch_RXTX(LOW);   //Turn-off transmitter, establish RX LOW

#ifdef DEBUG
   _INFOLIST("%s setup m(%d) slot(%d) f(%ld)\n",__func__,mode,Band_slot,freq);
#endif //DEBUG
}
/*--------------------------------------------------------------------------*
 * definePinOut
 * isolate pin definition on a board conditional procedure out of the main
 * setup() flow
 *--------------------------------------------------------------------------*/
void definePinOut() {


#ifdef ADX
   pinMode(UP,   INPUT);
   pinMode(DOWN, INPUT);
   pinMode(TXSW, INPUT);
   pinMode(RX,   OUTPUT);
   pinMode(WSPR, OUTPUT);
   pinMode(JS8,  OUTPUT);
   pinMode(FT4,  OUTPUT);
   pinMode(FT8,  OUTPUT);
   pinMode(TX,   OUTPUT);
   pinMode(AIN0, INPUT);  //PD6=AN0 must be grounded
   pinMode(AIN1, INPUT);  //PD7=AN1=HiZ

#ifdef ATUCTL
   pinMode(uint8_t(atu),  OUTPUT);
   flipATU();
#endif //ATUCTL

#ifdef DEBUG
   _INFO;
#endif //DEBUG
#endif //ADX

#ifdef PDX
   gpio_init(TX);
   gpio_init(UP);
   gpio_init(DOWN);
   gpio_init(TXSW);
   gpio_init(RX);
   gpio_init(WSPR);
   gpio_init(JS8);
   gpio_init(FT4);
   gpio_init(FT8);
   gpio_init(AIN0);
   gpio_init(AIN1);
   gpio_init(LED_BUILTIN); // for debugging

   gpio_set_dir(UP, GPIO_IN);
   gpio_set_dir(DOWN,GPIO_IN);
   gpio_set_dir(TXSW,GPIO_IN);

   gpio_pull_up(TXSW);
   gpio_pull_up(DOWN);
   gpio_pull_up(UP);

   gpio_set_dir(RX,GPIO_OUT);
   gpio_set_dir(TX, GPIO_OUT);
   gpio_set_dir(WSPR,GPIO_OUT);
   gpio_set_dir(JS8,GPIO_OUT);
   gpio_set_dir(FT4,GPIO_OUT);
   gpio_set_dir(FT8,GPIO_OUT);
   gpio_set_dir(LED_BUILTIN,GPIO_OUT);

   gpio_set_dir(AIN0,GPIO_IN);
   gpio_set_dir(AIN1,GPIO_IN);

#ifdef ATUCTL
   gpio_init(uint8_t(atu));
   gpio_set_dir (uint8_t(atu), GPIO_OUT);
   flipATU();
#endif //ATUCTL
#endif //PDX



}

/*---------------------------------------------------------------------------------------------
 * setup()
 * This is the main setup cycle executed once on the Arduino architecture
 *---------------------------------------------------------------------------------------------*/
void setup()
{

/*-----
 * Initialization is common for all uses of the serial port, specific variables and constants
 * has been given proper initialization based on the protocol used
 *-----*/

   #if (defined(DEBUG) || defined(CAT) || defined(TERMINAL))
      Serial.begin(BAUD);
      while (!Serial) {
      #ifdef WDT
         wdt_reset();
      #endif //WDT
      }
      Serial.flush();
      Serial.setTimeout(SERIAL_TOUT);
   #endif //DEBUG or CAT or Terminal

   #ifdef DEBUG
      #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
          const char * proc = "ATmega328P";
      #else
          const char * proc = "RP2040";
      #endif
      _EXCPLIST("%s: ADX Firmware V(%s) build(%d) board(%s)\n",__func__,VERSION,BUILD,proc);
   #endif //DEBUG

#ifdef CAT
   _EXCPLIST("%s CAT sub-system initialized\n",__func__);
#endif //CAT


   definePinOut();
   blinkLED(TX);

   setup_si5351();

   #ifdef DEBUG
      _EXCPLIST("%s setup_si5351 ok\n",__func__);
   #endif //DEBUG


#ifdef ADX
   PCICR  |= B00000100; // Enable interrupts at PD port
   PCMSK2 |= B00011100; // Signal interrupts for D2,D3 and D4 pins (UP/DOWN/TX)
   setWord(&button[INT0],PUSHSTATE,HIGH);
   setWord(&button[INT1],PUSHSTATE,HIGH);
   setWord(&button[INT2],PUSHSTATE,HIGH);

   #ifdef DEBUG
      _EXCPLIST("%s INT ok\n",__func__);
   #endif //DEBUG
#endif //ADX


   initADX();
   #ifdef DEBUG
      _EXCPLIST("%s INIT ok\n",__func__);
   #endif //DEBUG

   #ifdef QUAD
     setupQUAD();
     #ifdef DEBUG
        _EXCPLIST("%s setupQUAD ok\n",__func__);
     #endif //DEBUG

     /*---------
      * Initialize the QUAD board with the default band (at slot 0)
      *
      */
     uint16_t s=Bands[Band_slot];
     int q=band2QUAD(s);
     if (q != -1) {
        setQUAD(q);
     }
     #ifdef DEBUG
        _EXCPLIST("%s Bands[%d]=%d quad=%d\n",__func__,Band_slot,s,q);
     #endif //DEBUG

   #endif //QUAD


/*------
 * Check if calibration is needed
 */
   if (detectKey(DOWN,LOW,WAIT)==LOW) {Calibration();}

#ifdef ADX
/*--------------------------------------------------------*
 * initialize the timer1 as an analog comparator          *
 * this is the main feature of the VOX/Modulation scheme  *
 *--------------------------------------------------------*/
  TCCR1A = 0x00;
  TCCR1B = 0x01; // Timer1 Timer 16 MHz
  TCCR1B = 0x81; // Timer1 Input Capture Noise Canceller
  ACSR |= (1<<ACIC);  // Analog Comparator Capture Input
//--------------------------------------------------------*

  pinMode(AIN1, INPUT); //PD7 = AN1 = HiZ, PD6 = AN0 = 0

  #ifdef DEBUG
     _EXCPLIST("%s Timer1 set Ok\n",__func__);
  #endif //DEBUG
#endif //ADX

  switch_RXTX(LOW);
  #ifdef DEBUG
      _EXCPLIST("%s switch_RXTX Low ok\n",__func__);
  #endif //DEBUG

  Mode_assign();

  #ifdef WDT

     #ifdef ADX
        wdt_disable();
        wdt_enable(WDTO_8S);
     #endif //ADX

     #ifdef PDX
        watchdog_enable(8000, 1);
     #endif //PDX

     setWord(&TSW,TX_WDT,false);
  #endif //WDT

  #ifdef DEBUG
     _EXCPLIST("%s completed\n",__func__);
  #endif //DEBUG


#ifdef TERMINAL
/*------------------
 * if UP switch pressed at bootup then enter Terminal mode
 */
   if (detectKey(UP,LOW,WAIT)==LOW) {
      execTerminal();
   }
#endif //TERMINAL


}
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                   Board Main Dispatched and operational loop                                *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
void loop()
{

 //*--- Debug hook
    keepAlive();

//*--- changes in mode, band, frequency and operational status
    checkMode();

//*--- Manage anti-VOX timeout after avoxtime (mSec) the anti-vox condition is cleared

    #ifdef ANTIVOX

    if (getWord(TSW,AVOX)==true) {
       if (millis()-tavox > uint32_t(avoxtime)) {
          setWord(&TSW,AVOX,false);
          tavox=0;
       }
    }

    #endif //ANTIVOX

    #ifdef EE
//*--- if EEPROM enabled check if timeout to write has been elapsed
    checkEEPROM();
    #endif //EEPROM

    #ifdef ATUCTL

//*--- ATU pulse width control

    if ((millis()-tATU)>atu_delay && getWord(TSW,ATUCLK)==true) {
       setWord(&TSW,ATUCLK,false);
       setGPIO(atu,LOW);
    }
    #endif //ATUCTL

    #ifdef CAT
       serialEvent();
    #endif //CAT

    #ifdef WDT

       if ((millis() > (wdt_tout+uint32_t(WDT_MAX))) && getWord(SSW,TXON) == HIGH && getWord(SSW,CATTX)==true) {
          switch_RXTX(LOW);
          setWord(&TSW,TX_WDT,HIGH);
          wdt_tout=millis();
       }

//*--- if WDT enabled reset the watchdog
       wdt_reset();
    #endif //WDT

/*----------------------------------------------------------------------------------*
 * main transmission loop                                                           *
 * Timer1 (16 bits) with no pre-scaler (16 MHz) is checked to detect zero crossings *
 * if there is no overflow the frequency is calculated                              *
 * if activity is detected the TX is turned on                                      *
 * TX mode remains till no further activity is detected (operate like a VOX command)*
 *----------------------------------------------------------------------------------*/
#ifdef ADX

uint16_t n = VOX_MAXTRY;
    setWord(&SSW,VOX,false);
    while ( n > 0 ){                                 //Iterate up to 10 times looking for signal to transmit

    #ifdef WDT
       wdt_reset();

       if (getWord(TSW,TX_WDT)==HIGH) {
           break;
       }  //If watchdog has been triggered so no TX is allowed till a wdt_max timeout period has elapsed

       if ((millis() > (wdt_tout+uint32_t(WDT_MAX))) && getWord(SSW,TXON) == HIGH) {
          switch_RXTX(LOW);
          setWord(&TSW,TX_WDT,HIGH);
          wdt_tout=millis();
          break;
       }

    #endif //WDT

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
       //if ((d2-d1) == 0) break;
       unsigned long codefreq = CPU_CLOCK/(d2-d1);

       if ((codefreq < FRQ_MAX) && (codefreq > 0)){

#ifdef ANTIVOX
          if (getWord(TSW,AVOX)==false) {
#endif //ANTIVOX

             if (getWord(SSW,VOX) == false){
                 switch_RXTX(HIGH);
             }

             si5351.set_freq(((freq + codefreq) * 100ULL), SI5351_CLK0);
             setWord(&SSW,VOX,true);

#ifdef ANTIVOX
          }  else {
            if (millis()-tavox > uint32_t(avoxtime)) {
               setWord(&TSW,AVOX,false);
               tavox=0;
            }
          }
#endif //ANTIVOX

       #ifdef WDT
          wdt_reset();
       #endif //WDT

       }
    } else {
       n--;
    }


    #ifdef CAT
//*--- if CAT enabled check for serial events (again)
       serialEvent();
    #endif

    #ifdef WDT
       wdt_reset();
    #endif //WDT
 }
#endif //ADX
/*---------------------------------------------------------------------------------*
 * when out of the loop no further TX activity is performed, therefore the TX is   *
 * turned off and the board is set into RX mode                                    *
 *---------------------------------------------------------------------------------*/
#ifdef WDT

/*-----------------------------------------------------------*
 * Check for watchdog                                        *
 * if activated blink TX LED and wait till a full timeout to *
 * restore the TX capability.                                *
 * If the continuous TX condition persists (i.e. noise at the*
 * SPKR input at least some cooling time was allowed, if     *
 * the watchdog was activated by a wrong CAT command then    *
 * it will stay ready for the next TX command                *
 *                                                           *
 *-----------------------------------------------------------*/
 if (getWord(TSW,TX_WDT)==HIGH) {
    blinkLED(TX);
 }

 if (getWord(SSW,TXON)==LOW && getWord(TSW,TX_WDT)==HIGH && (millis() > (wdt_tout+uint32_t(WDT_MAX)))) {
    setWord(&TSW,TX_WDT,LOW);   //Clear watchdog condition
 }

#endif //WDT

#ifdef CAT
 serialEvent();
#endif //CAT

 if (getWord(SSW,CATTX)!=true) {
    switch_RXTX(LOW);
    setWord(&SSW,VOX,false);
    setWord(&SSW,TXON,false);
 }

 #ifdef WDT
    wdt_reset();
 #endif //WDT

}

//****************************[ END OF MAIN LOOP FUNCTION ]*******************************
//********************************[ END OF FIRMWARE ]*************************************
