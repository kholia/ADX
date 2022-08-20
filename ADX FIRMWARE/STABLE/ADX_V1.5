
//*********************************************************************************************************
//********************* ADX - ARDUINO based DIGITAL MODES 4 BAND HF TRANSCEIVER ***************************
//********************************* Write up start: 02/01/2022 ********************************************
// FW VERSION: ADX_QUAD_V1.1 - Version release date: 04/11/2022
// Barb(Barbaros ASUROGLU) - WB2CBA - 2022
//*********************************************************************************************************
// FW VERSION: ADX_QUAD_V1.2e (Experimental) release date 01/jun/2022
// PEC (Dr. Pedro E. Colla) - LU7DZ - 2022
// This is an experimental version implementing the following features over the standard ADX firmware
//     - CAT support (TS480 protocol, limited implementation)
//     - CW support (no envelop shape control, see release notes)
//     - EEPROM optimized read/write
//     - Hardware watchdog
//     - Few minor code optimizations
//     - Frequency definitions for all HF bands
//*********************************************************************************************************
// FW VERSION: ADX_QUAD_V1.3e (Experimental) release data 22/jun/2022
// PEC (Dr. Pedro E. Colla) - LU7DZ - 2022
// This is an experimental version implementing the following features over the 1.2e experimental version
//     - Support for the QUAD (4 bands) PA & LPF daughter board
//     - Support for an external ATU (control signal to reset on band changes), D5 line used for this control
//     - Other minor code optimizations & bug fixing
//*********************************************************************************************************
// FW VERSION: ADX_QUAD_V1.4e (Experimental) release data 23/jun/2022
// PEC (Dr. Pedro E. Colla) - LU7DZ - 2022
// This is an experimental version implementing the following features over the 1.3e experimental version
//     - Support for the CAT Protocol for the ICOM-746 Rig (Original code by Alan Altman (AG7XS) on his ADX_CAT_V3.51.ino version
//     - Other minor code optimizations & bug fixing
//*********************************************************************************************************
// FW VERSION: ADX_QUAD_V1.5e (Experimental) release date 30/jun/2022
// PEC (Dr. Pedro E. Colla) - LU7DZ - 2022
// This is an experimental version implementing the following features over the 1.4e experimental version
//     - Enhanced watchdog, now if the TX state last over 2 minutes the transmitter is turned off
//     - Serial configuration facility
//     - Other minor code optimizations & bug fixing
//*********************************************************************************************************
// FW VERSION: ADX_QUAD_V1.5 (Baseline) release date 16-Aug-2022
// Barb (WB2CBA), Dhiru (VU3CER) & Pedro (LU7DZ)
// Release version
//     - Enhanced EEPROM management (EE)
//     - Watchdog (WDT)
//     - CAT (TS840 protocol)
//     - Support for QUAD multiband board (QUAD)
//     - ATU reset control (optional).
//     - CW mode (optional)
//***********************************************************************************************************************
// Required Libraries
// ----------------------------------------------------------------------------------------------------------------------
// Etherkit Si5351 (Needs to be installed via Library Manager to arduino ide) - 
//          SI5351 Library by Jason Mildrum (NT7S) - https://github.com/etherkit/Si5351Arduino
// Arduino "Wire.h" I2C library(built-into arduino ide)
// Arduino "EEPROM.h" EEPROM Library(built-into arduino ide)
// AVR "wdt.h" Watchdog Library
//*************************************[ LICENCE and CREDITS ]*********************************************
//  FSK TX Signal Generation code by: Burkhard Kainka(DK7JD) - http://elektronik-labor.de/HF/SDRtxFSK2.html
//  SI5351 Library by Jason Mildrum (NT7S) - https://github.com/etherkit/Si5351Arduino
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

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                            EXTERNAL LIBRARIES USED                                          *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
/*-------------------------------------------------------------*
 * Define the runtime platform either PICO (Raspberry Pi Pico) *
 * or !PICO (Arduino ATMega328p)                               *
 *-------------------------------------------------------------*/
#define ADX              1   //This is the standard ADX Arduino based board 
 
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                            EXTERNAL LIBRARIES USED                                          *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*

#include <Arduino.h>
#include <stdint.h>
#include <si5351.h>
#include "Wire.h"
#include <EEPROM.h>  
#include <avr/wdt.h> 

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                            VERSION HEADER                                                   *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
#define VERSION        "1.5e"
#define BUILD          220

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                            MACRO DEFINES                                                    *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
#define BOOL2CHAR(x)  (x==true ? "True" : "False")
#undef  _NOP
#define _NOP          (byte)0

void(* resetFunc) (void) = 0;  // declare reset fuction at address 0 //resetFunc(); to reboot
#define getGPIO(x) digitalRead(x) 
#define setGPIO(x,y) digitalWrite(x,y)  

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                            FEATURE CONFIGURATION PROPERTIES                                 *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*

   #define WDT            1      //Hardware and TX watchdog enabled
   #define EE             1      //User EEPROM for persistence
   #define CAT            1      //Enable CAT protocol over serial port
   #define TS480          1      //CAT Protocol is Kenwood 480
   #define QUAD           1      //Enable the usage of the QUAD 4-band filter daughter board
   #define ATUCTL         1      //Control external ATU device
/*
 * The following definitions are disabled but can be enabled selectively
 */

   //#define RESET          1      //Allow a board reset (*)-><Band Select> -> Press & hold TX button for more than 2 secs will reset the board (EEPROM preserved)
   //#define ANTIVOX        1      //Anti-VOX enabled, VOX system won't operate for AVOXTIME mSecs after the TX has been shut down by the CAT system
   //#define ONEBAND        1      //Forces a single band operation in order not to mess up because of a wrong final filter    
   //#define CW             1      //CW support
   //#define CAL_RESET      1      //If enabled reset cal_factor when performing a new calibration()
   //#define DEBUG          1      //DEBUG turns on different debug, information and trace capabilities, it is nullified when CAT is enabled to avoid conflicts
   //#define FT817          1      //CAT Protocol is FT 817


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
#define DELAY_WAIT     BDLY*2       //Double Delay
#define DELAY_CAL      DELAY_WAIT/10
#define MAXMODE        5            //Max number of digital modes
#define MAX_BLINK      4            //Max number of blinks
#define BANDS          4            //Max number of bands allowed
#define MAXBAND       9            //Max number of bands defined (actually uses BANDS out of MAXBAND)
#define XT_CAL_F      33000         //Si5351 Calibration constant 
#define CAL_STEP      500           //Calibration factor step up/down while in calibration (sweet spot experimentally found by Barb)
#define REPEAT_KEY    30            //Key repetition period while in calibration
#define WAIT          true          //Debouncing constant
#define NOWAIT        false         //Debouncing constant
#define SERIAL_TOUT   50
#define SERIAL_WAIT   2
#define CAT_RECEIVE_TIMEOUT      500

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                      PIN ASSIGNMENTS                                                        *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
   #define UP              2           //UP Switch
   #define DOWN            3           //DOWN Switch
   #define TXSW            4           //TX Switch

   #define AIN0            6           //(PD6)
   #define AIN1            7           //(PD7)

   #define RX              8           //RX Switch
   #define TX             13           //(PB5) TX LED

   #define WSPR            9           //WSPR LED 
   #define JS8            10           //JS8 LED
   #define FT4            11           //FT4 LED
   #define FT8            12           //FT8 LED
#ifdef ATUCTL
   #define ATU             5       //ATU Device control line (flipped HIGH during 200 mSecs at a band change)
#endif //ATUCTL

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                      GLOBAL STATE VARIABLE DEFINITIONS                                      *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
/*----------------------------------------------------------------*
 *  Global State Variables (Binary)                               *
 *----------------------------------------------------------------*/
#define TXON        0B00000001    //State of the TX
#define VOX         0B00000010    //Audio input detected
#define UPPUSH      0B00000100    //UP button pressed
#define DNPUSH      0B00001000    //DOWN button pressed
#define TXPUSH      0B00010000    //TXSW button pressed
#define CATTX       0B00100000    //TX turned on via CAT (disable VOX)
#define SAVEEE      0B01000000    //Mark of EEPROM updated
#define CWMODE      0B10000000    //CW Mode active
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
#define UNUSED      0B10000000    //Counter mode semaphore

/*----------------------------------------------------------------*
 * IPC Management                                               *
 * ---------------------------------------------------------------*/
#define QWAIT       0B00000001    //Semaphore Wait
#define QCAL        0B00000010    //Calibration (using 2 cores)
#define QFSK        0B00000100    //FSK detection
#define QDATA       0B00001000    //FSK new datum
#define FSKMIN             200    //Minimum FSK frequency computed
#define FSKMAX            2500    //Maximum FSK frequency computed

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
   

//*--- If a QUAD multiband board is defined then the transceiver must not be a single band one
   
#if (defined(QUAD))
    #undef   ONEBAND
#endif //QUAD, no ONEBAND    

//*--- If CAT is defined then DEBUG  can not be activated simultaneously

#if (defined(CAT) && defined(DEBUG))  //Rule for conflicting usage of the serial port
   #undef  DEBUG
#endif // CAT && DEBUG

//*--- If CAT is not defined then erase all conflicting definitions

#if (!defined(CAT))  //Rule for conflicting usage of the CAT Protocol (can't activate extended without basic)
   #undef  TS480
   #undef  FT817
#endif // CAT && DEBUG

//*--- if both supported CAT protocols are simultaneously selected then keep one

   

#ifdef TS480
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*               DEFINITIONS SPECIFIC TO TS480 CAT PROTOCOL                                    *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*

   #define CATCMD_SIZE          18
   
   volatile char    CATcmd[CATCMD_SIZE];
   const int        BUFFER_SIZE = CATCMD_SIZE;
   char             buf[BUFFER_SIZE];

#endif //TS480

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*               DEBUG SUPPORT MACRO DEFINITIONS                                               *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*

/*****************************************************************
 * Trace and debugging macros (only enabled if DEBUG is set      *
 *****************************************************************/
//#define DEBUG  1
#ifdef DEBUG        //Remove comment on the following #define to enable the type of debug macro
   //#define INFO  1   //Enable _INFO and _INFOLIST statements
   //#define EXCP  1   //Enable _EXCP and _EXCPLIST statements
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


#ifdef CW
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*       CW FEATURE IF PTT IS ACTIVATED BY BUTTON/CAT THE TX FREQUENCY WILL BE SHIFT UP        *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
   #define CWSHIFT       600
   uint16_t cwshift=CWSHIFT;
#endif //CW

#ifdef EE
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*       DEFINITIONS RELATED TO THE USAGE OF EEPROM AS A PERMANENT STORAGE                     *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
   #define EEPROM_CAL          10
   #define EEPROM_BUILD        20
   #define EEPROM_TEMP         30
   #define EEPROM_MODE         40
   #define EEPROM_BAND         50

   uint32_t tout=0;

   //#define EEPROM_CLR     1   //Initialize EEPROM (only to be used to initialize contents)
   #define EEPROM_SAVED   100   //Signature of EEPROM being updated at least once
   #define EEPROM_TOUT  10000   //Timeout in mSecs to wait till commit to EEPROM any change
#endif //EEPROM

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*       GLOBAL VARIABLE DEFINITION                                                            *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
uint16_t bounce_time    = BOUNCE_TIME;
uint16_t short_time     = SHORT_TIME;
uint16_t vox_maxtry     = VOX_MAXTRY;
int      cnt_max        = CNT_MAX;
uint16_t max_blink      = MAX_BLINK;
uint8_t  SSW            = 0;          //System SSW variable (to be used with getWord/setWord)
uint8_t  TSW            = 0;          //System timer variable (to be used with getWord/setWord);
uint16_t mode           = 0;          //Default to mode=0 (FT8)
uint16_t Band_slot      = 0;          //Default to Bands[0]=40
int32_t  cal_factor     = 0;
unsigned long Cal_freq  = 1000000UL; // Calibration Frequency: 1 Mhz = 1000000 Hz
unsigned long f[MAXMODE]                  = { 7074000, 7047500, 7078000, 7038600, 7030000};   //Default frequency assignment   
const unsigned long slot[MAXBAND][MAXMODE]={{ 3573000, 3575000, 3578000, 3568600, 3560000},   //80m [0]
                                            { 5357000, 5357000, 5357000, 5287200, 5346500},   //60m [1] 
                                            { 7074000, 7047500, 7078000, 7038600, 7030000},   //40m [2]
                                            {10136000,10140000,10130000,10138700,10106000},   //30m [3]
                                            {14074000,14080000,14078000,14095600,14060000},   //20m [4]
                                            {18100000,18104000,18104000,18104600,18096000},   //17m [5]
                                            {21074000,21140000,21078000,21094600,21060000},   //15m [6]  
                                            {24915000,24915000,24922000,24924600,24906000},   //12m [7]                                                                     
                                            {28074000,28074000,28078000,28124600,28060000}};  //10m [8]                           
                                                     
unsigned long freq      = f[mode]; 
const uint8_t LED[4]    = {FT8,FT4,JS8,WSPR};  //A 5th virtual mode is handled if CW enabled, LEDS are managed in that case not using this table
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*       GLOBAL VARIABLE DEFINITION CONDITIIONAL TO DIFFERENT OPTIONAL FUNCTIONS               *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*

#ifdef EE
uint16_t eeprom_tout = EEPROM_TOUT;
#endif //EE


#if (defined(ATUCTL) || defined(WDT))
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
      _EXCP;
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
  _EXCPLIST("%s band=%d quad=%d\n",__func__,b,q);
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
      _EXCPLIST("%s() LPF=%d QUAD=%d\n",__func__,LPFslot,s);
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
      _EXCP;
   #endif //DEBUG
  
}
#endif //QUAD

#ifdef CAT
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
   if (f>=24890000 && f<24990000) {b=12;}
   if (f>=28000000 && f<29700000) {b=10;}

#ifdef DEBUG
   _EXCPLIST("%s() f=%ld band=%d\n",__func__,f,b);
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
   _EXCPLIST("%s() band=%d slot=%d\n",__func__,band,s);
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

   return s;
 
}
/*-----------------------------------------------------------------*
 * getMode                                                         *
 * given the slot in the slot[][] array and the frequency returns  *
 * the mode that should be assigned, -1 if none can be identified  *
 *-----------------------------------------------------------------*/
int getMode(int s,uint32_t f) {
  
  int m=-1;
  for (int i=0;i<MAXMODE-1;i++) {
    if (slot[s][i]==f) {
       m=i;
       break;
    }
  }
  
  #ifdef DEBUG
  _EXCPLIST("%s slot=%d f=%ld m=%d\n",__func__,s,f,m);
  #endif //DEBUG
  
  return m;
}

#ifdef FT817

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                   FT817 CAT PROTOCOL SUBSYSTEM                                              *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
/**
   The CAT protocol is used by many radios to provide remote control to computers through
   the serial port.

   This is very much a work in progress. Parts of this code have been liberally
   borrowed from other GPL licensed works like hamlib.

   https://github.com/afarhan/ubitxv6/blob/master/ubitx_cat.cpp

   Note: This code was tested with WSJT-X 2.5.4 and Hamlib (rigctl) 4.3.1 in
   July-2022 by Dhiru (VU3CER).

   Reference: http://www.ka7oei.com/ft817_meow.html
   
*/

/*---
 * Protocol constant definitions
 *---*/
#define CAT_MODE_LSB            0x00
#define CAT_MODE_USB            0x01
#define CAT_MODE_CW             0x02
#define CAT_MODE_CWR            0x03
#define CAT_MODE_AM             0x04
#define CAT_MODE_FM             0x08
#define CAT_MODE_DIG            0x0A
#define CAT_MODE_PKT            0x0C
#define CAT_MODE_FMN            0x88
#define ACK                     0x00


unsigned char doingCAT = 0;
bool txCAT             = false;        // turned on if the transmitting due to a CAT command
char inTx              = 0;                // it is set to 1 if in transmit mode (whatever the reason : cw, ptt or cat)
char isUSB             = 0;

static unsigned long rxBufferArriveTime = 0;
static byte rxBufferCheckCount          = 0;
static byte cat[5];
static byte insideCat                   = 0;
unsigned int skipTimeCount              = 0;

/*---
 * Nibble Format routines
 * --*/
byte setHighNibble(byte b, byte v) {
  // Clear the high nibble
  b &= 0x0f;
  // Set the high nibble
  return b | ((v & 0x0f) << 4);
}

byte setLowNibble(byte b, byte v) {
  // Clear the low nibble
  b &= 0xf0;
  // Set the low nibble
  return b | (v & 0x0f);
}

byte getHighNibble(byte b) {
  return (b >> 4) & 0x0f;
}

byte getLowNibble(byte b) {
  return b & 0x0f;
}
/*----
  Takes a number and produces the requested number of decimal digits, staring
  from the least significant digit.
 *----*/ 
void getDecimalDigits(unsigned long number, byte* result, int digits) {
  for (int i = 0; i < digits; i++) {
    // "Mask off" (in a decimal sense) the LSD and return it
    result[i] = number % 10;
    // "Shift right" (in a decimal sense)
    number /= 10;
  }
}
/*---
  Takes a frequency and writes it into the CAT command buffer in BCD form.
 *---*/ 
void writeFreq(unsigned long freq, byte* cmd) {
  // Convert the frequency to a set of decimal digits. We are taking 9 digits
  // so that we can get up to 999 MHz. But the protocol doesn't care about the
  // LSD (1's place), so we ignore that digit.
  byte digits[9];
  getDecimalDigits(freq, digits, 9);
  
  // Start from the LSB and get each nibble
  
  cmd[3] = setLowNibble(cmd[3],  digits[1]);
  cmd[3] = setHighNibble(cmd[3], digits[2]);
  cmd[2] = setLowNibble(cmd[2],  digits[3]);
  cmd[2] = setHighNibble(cmd[2], digits[4]);
  cmd[1] = setLowNibble(cmd[1],  digits[5]);
  cmd[1] = setHighNibble(cmd[1], digits[6]);
  cmd[0] = setLowNibble(cmd[0],  digits[7]);
  cmd[0] = setHighNibble(cmd[0], digits[8]);
}
/*---
// This function takes a frequency that is encoded using 4 bytes of BCD
// representation and turns it into an long measured in Hz.
//
// [12][34][56][78] = 123.45678? Mhz
*----*/
unsigned long readFreq(byte* cmd) {
  // Pull off each of the digits
  byte d7 = getHighNibble(cmd[0]);
  byte d6 = getLowNibble(cmd[0]);
  byte d5 = getHighNibble(cmd[1]);
  byte d4 = getLowNibble(cmd[1]);
  byte d3 = getHighNibble(cmd[2]);
  byte d2 = getLowNibble(cmd[2]);
  byte d1 = getHighNibble(cmd[3]);
  byte d0 = getLowNibble(cmd[3]);
  return
    (unsigned long)d7 * 100000000L +
    (unsigned long)d6 * 10000000L +
    (unsigned long)d5 * 1000000L +
    (unsigned long)d4 * 100000L +
    (unsigned long)d3 * 10000L +
    (unsigned long)d2 * 1000L +
    (unsigned long)d1 * 100L +
    (unsigned long)d0 * 10L;
}

/*---
 * This function is to falsify some readings performed
 * into the volatile memory of a typical FT817 radio 
 *---*/
void catReadEEPRom(void)
{
  byte temp0 = cat[0];
  byte temp1 = cat[1];
  cat[0] = 0;
  cat[1] = 0;

  switch (temp1)
  {
    case 0x45:
      if (temp0 == 0x03) {
        cat[0] = 0x00;
        cat[1] = 0xD0;
      }
      break;
    case 0x47: //
      if (temp0 == 0x03) {
        cat[0] = 0xDC;
        cat[1] = 0xE0;
      }
      break;
    case 0x55:
      // 0: VFO A/B  0 = VFO-A, 1 = VFO-B
      cat[1] = 0x00;
      break;
    case 0x57:
      cat[0] = 0xC0;
      cat[1] = 0x40;
      break;
    case 0x59:
      // http://www.ka7oei.com/ft817_memmap.html
      break;
    case 0x5C: // Beep Volume (0-100) (#13)
      cat[0] = 0xB2;
      cat[1] = 0x42;
      break;
    case 0x5E:
      cat[1] = 0x25;
      break;
    case 0x61: // Sidetone (Volume) (#44)
      cat[1] = 0x08;
      break;
    case 0x5F:
      cat[0] = 0x32;
      cat[1] = 0x08;
      break;
    case 0x60 : // CW Delay (10-2500 ms)
      // cat[0] = cwDelayTime;
      cat[1] = 0x32;
      break;
    case 0x62:
      cat[1] = 0xB2;
      break;
    case 0x63:
      cat[0] = 0xB2;
      cat[1] = 0xA5;
      break;
    case 0x64:
      break;
    case 0x67: // 6-0 SSB Mic (#46) Contains 0-100 (decimal) as displayed
      cat[0] = 0xB2;
      cat[1] = 0xB2;
      break;
    case 0x69: // FM Mic (#29) Contains 0-100 (decimal) as displayed
      break; // XXX
    case 0x78:
      if (isUSB)
        cat[0] = CAT_MODE_USB;
      else
        cat[0] = CAT_MODE_LSB;
      if (cat[0] != 0) cat[0] = 1 << 5;
      break;
    case 0x79:
      cat[0] = 0x00;
      cat[1] = 0x00;
      break;
    case 0x7A: // SPLIT
      break;
    case 0xB3:
      cat[0] = 0x00;
      cat[1] = 0x4D;
      break;

  }

  // send the data
  
  Serial.write(cat, 2);
  
  #ifdef ADX
     delay(SERIAL_WAIT);
     Serial.flush();
     delay(50);
  #endif //ADX   

}
/*---
 * Main FT817 CAT protocol command processor and dispatcher
 *----*/
void processCATCommand2(byte* cmd) {
  byte response[5];
  unsigned long f;

  switch (cmd[4]) {
    case 0x01:   // set frequency
    {
      f = readFreq(cmd);
      freq=f;

      response[0] = 0;
      Serial.write(response, 1);
      delay(SERIAL_WAIT);
      Serial.flush();
      delay(50);
      break;
  }
    case 0x02: // split on
    {
      break;
    }  
    case 0x82: // split off
    {
      break;
    }
    case 0x03: 
    {
      unsigned long fx=freq;
      writeFreq(fx, response); // Put the frequency into the buffer
      if (isUSB) {
        response[4] = 0x01; // USB
      } else {
        response[4] = 0x00; // LSB
      }  
      Serial.write(response, 5);
      delay(SERIAL_WAIT);
      Serial.flush();
      delay(50);

      break;
    }
    case 0x07: // set mode
      {
      if (cmd[0] == 0x00 || cmd[0] == 0x03) {
        isUSB = 0;
      } else {
        isUSB = 1;
      }  
      response[0] = 0x00;
      Serial.write(response, 1);
      delay(SERIAL_WAIT);
      Serial.flush();
      delay(50);
      break;
      }
    case 0x08: // PTT On
    {
      if (!inTx) {
        response[0] = 0;
        inTx = 1;
        setWord(&SSW,CATTX,true);
        switch_RXTX(HIGH);
      } else {
        response[0] = 0xf0;
      }
      Serial.write(response, 1);
      delay(SERIAL_WAIT);
      Serial.flush();
      delay(50);

      break;
    }
    case 0x88: // PTT Off
    {
      if (inTx) {
        inTx = 0;
        setWord(&SSW,CATTX,false);
        switch_RXTX(LOW);
      }
      response[0] = 0;
      Serial.write(response, 1);
      delay(SERIAL_WAIT);
      Serial.flush();
      delay(50);
      break;
    }
    case 0x81: // toggle the VFOs
  {
      response[0] = 0;
      Serial.write(response, 1);
      delay(SERIAL_WAIT);
      Serial.flush();
      delay(50);

      break;
  }
    case 0xBB: // Read FT-817 EEPROM Data
  {
      catReadEEPRom();
      break;
  }
    case 0xe7:
       {
      // Get receiver status, we have hardcoded this as
      // as we don't support ctcss, etc.
      response[0] = 0x09;
      Serial.write(response, 1);
      delay(SERIAL_WAIT);
      Serial.flush();
      delay(50);

      break;
       }
    case 0xf7:
      {
        boolean isHighSWR = false;
        boolean isSplitOn = false;
        response[0] = ((inTx ? 0 : 1) << 7) +
                      ((isHighSWR ? 1 : 0) << 6) + // Hi swr off / on
                      ((isSplitOn ? 1 : 0) << 5) + // Split on / off
                      (0 << 4) + // dummy data
                      0x08; // P0 meter data
        Serial.write(response, 1);
        delay(SERIAL_WAIT);
        Serial.flush();
        delay(50);

      }
      break;

    default:
    {
      response[0] = 0x00;
      Serial.write(response[0]);
      delay(SERIAL_WAIT);
      Serial.flush();
      delay(50);
    }
  }
  insideCat = false;
}
/*---
 * serialEvent() handler
 *---*/
void serialEvent() {
  byte i;

  // Check Serial Port Buffer
  if (Serial.available() == 0) {                            // Set Buffer Clear status
    rxBufferCheckCount = 0;
    return;
  }
  else if (Serial.available() < 5) {                        // First Arrived
    if (rxBufferCheckCount == 0) {
      rxBufferCheckCount = Serial.available();
      rxBufferArriveTime = millis() + CAT_RECEIVE_TIMEOUT;  // Set time for timeout
    }
    else if (rxBufferArriveTime < millis()) {               // Clear Buffer
      for (i = 0; i < Serial.available(); i++)
        rxBufferCheckCount = Serial.read();
      rxBufferCheckCount = 0;
    }
    else if (rxBufferCheckCount < Serial.available()) {     // Increase buffer count, slow arrive
      rxBufferCheckCount = Serial.available();
      rxBufferArriveTime = millis() + CAT_RECEIVE_TIMEOUT;  // Set time for timeout
    }
    return;
  }

  // CAT DATA arrived
  for (i = 0; i < 5; i++)
    cat[i] = Serial.read();

  // Note: This code is not re-entrant!
  if (insideCat == 1)
    return;
  insideCat = 1;

  processCATCommand2(cat);
  insideCat = 0;
}
#endif //FT817


//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                   TS480 CAT PROTOCOL SUBSYSTEM                                              *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*

#ifdef TS480
/*-----------------------------------------------------------------------------------------------------*
 *                                    TS480 CAT SubSystem                                              *
 * cloned from uSDX (QCX-SSB) firmware, this is a very large and complex yet very complete CAT protocol*                                    
 * because of memory constraints it has been implemented in two sections, a basic one which is the set *
 * needed to interact with WSJT-X (or FLRig) and a full protocol (activated with the CAT_FULL directive* 
 * intended to comply with most if not all the protocol commands, many answers aren't actually other   *
 * than a hardwired response not reflecting the actual status of the ADX transceiver, at the same time *
 * the ADX transceiver has less features than a real TS-480 rig, thus many commands refers to features *
 * which aren't really present on the ADX so nothing else than a hard wired command can be given, still*
 * many program requires some response to commands, even a phony one.                                  *
 * Adaptations and extensions by P.E.Colla (LU7DZ)                                                     *
 * ----------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------*
   CAT support inspired by Charlie Morris, ZL2CTM, contribution by Alex, PE1EVX, 
   source: http://zl2ctm.blogspot.com/2020/06/digital-modes-transceiver.html?m=1
   https://www.kenwood.com/i/products/info/amateur/ts_480/pdf/ts_480_pc.pdf
   Code excerpts from QCX-SSB by Guido (PE1NNZ)
   Mods by Pedro E. Colla(LU7DZ) 2022
 *----------------------------------------------------------------------------------------------------*/

  const char *cID ="ID";    
  const char *cIDr="ID020;";
  const char *cRX0="RX0;";
  const char *cTX0="TX0;";
  const char *cMD2="MD2;";
  const char *cMD3="MD3;";
  

/*-------------------------------------------------------------*
 * Specific CAT commands implementation                        *
 *-------------------------------------------------------------*/
 
//*--- Get Freq VFO A
void Command_GETFreqA()          //Get Frequency VFO (A)
{
  sprintf(hi,"FA%011ld;",freq);
  Serial.print(hi);
}

//*--- Set Freq VFO A
void setFreqCAT() {
  char Catbuffer[16];
  for (int i=2;i<13;i++) {
    Catbuffer[i-2]=CATcmd[i];
  }
  Catbuffer[11]='\0';
  uint32_t fx=(uint32_t)(atol(Catbuffer)*1000/1000);
  int      b=setSlot(fx);

  if (b<0) {
    return;
  }
  
  freq=fx;

 /*--- 
  * If a band change is detected switch to the new band
  *---*/
    
  if (b!=Band_slot) { //band change
     Band_slot=b;
     Freq_assign();
     freq=fx;
  }

  /*---
   * Properly register the mode if the frequency implies a WSJT mode change (FT8,FT4,JS8,WSPR) ||
   */
     
    
  int i=getBand(freq);
  if ( i<0 ) {
     return;
  }
  int j=findSlot(i);
  if (j<0 || j>3) {
     return;
  }
  int k=Bands[j];
  int q=band2Slot(k);
  int m=getMode(q,freq);

#ifdef DEBUG
  _EXCPLIST("%s f=%ld band=%d slot=%d Bands=%d b2s=%d m=%d mode=%d\n",__func__,freq,i,j,k,q,m,mode);
#endif //DEBUG  

  if (getWord(SSW,CWMODE)==false) {   
  
     if (mode != m) {
        mode = m;
        Mode_assign();
     }
  }
/*----
 * if enabled change filter from the LPF filter bank
 *----*/
  
  #ifdef QUAD  //Set the PA & LPF filter board settings if defined
     int x=band2QUAD(k);
     if (x != -1) {
        setQUAD(x);
     }   
  #endif //PALPF    

  #ifdef DEBUG
      _EXCPLIST("%s() CAT=%s f=%ld slot=%d bands[]=%d slot=%d quad=%d\n",__func__,Catbuffer,freq,b,k,q,x);
  #endif //DEBUG 
}

void Command_SETFreqA()          //Set Frequency VFO (A)
{
  setFreqCAT();
  Command_GETFreqA();
}

//*--- Get Freq VFO B
void Command_GETFreqB()          //Get Frequency VFO (B) -- fallback to VFO (A) until implementation of VFOA/B pair
{
  sprintf(hi,"FB%011ld;",freq);
  Serial.print(hi);
}
//*--- Set Freq VFO B
void Command_SETFreqB()          //Set Frequency VFO (B) -- fallback to VFO (B) until implementation for VFOA/B pair
{
  setFreqCAT();
  Command_GETFreqB();
}

//*--- Prepare and send response to IF; command
void Command_IF()               //General purpose status information command (IF), needs to be modified if RIT is implemented
{
  sprintf(hi,"IF%011ld00000+000000000%c%c0000000;",freq,txstatus(),modeTS480());   //Freq & mode the rest is constant (fake)
  Serial.print(hi);
}

//*--- Get current Mode (only 2=USB and 3=CW are supported)

void Command_GetMD()      //Get MODE command, only USB (2) and CW (3) are supported, 4 digital modes (mode 0,1,2,3 are mapped as USB)
{
  sprintf(hi,"MD%c;",modeTS480());
  Serial.print(hi);
}

void Command_SetMD()      //Set MODE command, only USB (2) and CW (3) are supported
{
  if (CATcmd[2] != '3') {
     setWord(&SSW,CWMODE,false);
     mode=0;
     Mode_assign();
     sprintf(hi,"%s",cMD2);    // at this time only USB is allowed, needs to be modified when CW is added
             
  }
  Serial.print(hi);
}

//*--- Place transceiver in RX mode
void Command_RX()
{
  setWord(&SSW,CATTX,false); 

  switch_RXTX(LOW);
  setWord(&SSW,CATTX,false);
  sprintf(hi,"%s",cRX0);
  Serial.print(hi);
  #ifdef ANTIVOX
      tavox=millis();
      setWord(&TSW,AVOX,true);
  #endif //ANTIVOX   
}

//*--- Place transceiver in TX mode
void Command_TX()
{

  switch_RXTX(HIGH);
  setWord(&SSW,CATTX,true);
  sprintf(hi,"%s",cTX0);
  Serial.print(hi);
}


//*---- Translate ADX mode into the TS-480 coding for mode
char modeTS480() {

  if (mode==4) {
     return '3';
  }
  return '2';

}
char txstatus() {
  
  if (getWord(SSW,CATTX)==false) {
     return '0';
  }
  return '1';

  
}
/*---------------------------------------------------------------------------------------------
 *  CAT Main command parser and dispatcher
 *---------------------------------------------------------------------------------------------*/
void analyseCATcmd()
{
  char strcmd[4];

  
  if ((CATcmd[0] == 'F') && (CATcmd[1] == 'A') && (CATcmd[2] == ';'))  {Command_GETFreqA(); return;}
  if ((CATcmd[0] == 'F') && (CATcmd[1] == 'A') && (CATcmd[13] == ';')) {Command_SETFreqA(); return;}
  if ((CATcmd[0] == 'F') && (CATcmd[1] == 'B') && (CATcmd[2] == ';'))  {Command_GETFreqB(); return;}
  if ((CATcmd[0] == 'F') && (CATcmd[1] == 'B') && (CATcmd[13] == ';')) {Command_SETFreqB(); return;}
  if ((CATcmd[0] == 'I') && (CATcmd[1] == 'F') && (CATcmd[2] == ';'))  {Command_IF(); return;}
  if ((CATcmd[0] == 'M') && (CATcmd[1] == 'D') && (CATcmd[2] == ';'))  {Command_GetMD(); return;}
  if ((CATcmd[0] == 'M') && (CATcmd[1] == 'D') && (CATcmd[3] == ';'))  {Command_SetMD(); return;}
  if ((CATcmd[0] == 'R') && (CATcmd[1] == 'X'))                        {Command_RX(); return;}
  if ((CATcmd[0] == 'T') && (CATcmd[1] == 'X'))                        {Command_TX(); return;}
 
  strcmd[0]=CATcmd[0];
  strcmd[1]=CATcmd[1];
  strcmd[2]=0x00;

  if (strcmp(strcmd,cID)==0)                                                      {sprintf(hi,"%s",cIDr);Serial.print(hi);return;}
  Serial.print("?;");
}

/*-----------------------------------------------------------------*
 * serialEvent                                                     *
 * Process incoming characters from the serial buffer assemble     *
 * commands and process responses according with the TS480 cat prot*
 *-----------------------------------------------------------------*/
volatile uint8_t cat_ptr = 0;
volatile char serialBuffer[CATCMD_SIZE];

void serialEvent(){

  char strCmd[7];
  char const *strResp="RX0;ID020;";
  sprintf(strCmd,"RX;ID;");
  
  int nread=Serial.available();
  if (nread<=0) return;

  int rc=Serial.readBytes(buf,nread);
  if (rc<=0) {return;}
  buf[rc]=0x0;
  int k=0;
  for (int j=0;j<rc;j++){
    if (buf[j]!=0x0d && buf[j]!=0x0a) { 
        serialBuffer[k++]=buf[j];
    }    

  }
  
#ifdef DEBUG  
  _TRACELIST("%s CAT received buffer=%s len=%d\n",__func__,serialBuffer,rc);
#endif //DEBUG  

  if (strcmp((const char*)serialBuffer,strCmd)==0) { //coincidence

#ifdef DEBUG
  _TRACELIST("%s Hit RX;ID; string\n",__func__);
#endif //DEBUG
     
     Serial.write(strResp,10);
     setWord(&SSW,CATTX,false);
     switch_RXTX(LOW);
     delay(SERIAL_WAIT);
  } else { 
    for (int i=0;i<k;i++) {
       char data=serialBuffer[i];
       CATcmd[cat_ptr++] = data;

#ifdef DEBUG
       _TRACELIST("%s data=%c CATcmd[%d]=%c\n",__func__,data,i,CATcmd[i]);
#endif //DEBUG

       if(data == ';'){      
         CATcmd[cat_ptr] = '\0'; // terminate the array
         cat_ptr = 0;            // reset for next CAT command

#ifdef DEBUG
        _TRACELIST("%s() cmd(%s)\n",__func__,CATcmd);
#endif //DEBUG

        analyseCATcmd();
        delay(SERIAL_WAIT);
        Serial.flush();
        delay(50);
        
      } else {
        if(cat_ptr > (CATCMD_SIZE - 1)){
           Serial.print("?;");  //Overrun, send error
           cat_ptr = 0;         //Overrun, cleanse buffer       
           Serial.flush();
           delay(50);
        }
      }  
   }   
 }
}
#endif //TS480
#endif //CAT

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
    _EXCP;
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
     _EXCPLIST("%s pin=%d\n",__func__,LEDpin);
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
   _EXCP;
   #endif //DEBUG   

 
}

/*-----
 * Set a particular LED ON
 */
void rstLED(uint8_t LEDpin,bool clrLED) {      //Turn-on LED {pin}
   
   (clrLED==true ? resetLED() : void(_NOP)); 
   setGPIO(LEDpin,LOW);

#ifdef DEBUG
   _EXCPLIST("%s(%d)\n",__func__,LEDpin);
#endif //DEBUG   

}

/*-----
 * Set a particular LED ON
 */
void setLED(uint8_t LEDpin,bool clrLED) {      //Turn-on LED {pin}
   
   (clrLED==true ? resetLED() : void(_NOP)); 
   setGPIO(LEDpin,HIGH);

#ifdef DEBUG
   _EXCPLIST("%s(%d)\n",__func__,LEDpin);
#endif //DEBUG   

}

/*-------
 * Blink a given LED
 */
void blinkLED(uint8_t LEDpin) {    //Blink 3 times LED {pin}

#ifdef DEBUG
   _EXCPLIST("%s (%d)\n",__func__,LEDpin);
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

   #ifdef DEBUG   
   _EXCP;
   #endif //DEBUG
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


/**********************************************************************************************/
/*                               PushButton Management                                        */
/**********************************************************************************************/
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
                 #ifdef DEBUG
                   _EXCPLIST("%s switch(%d) value(%s)\n",__func__,k,BOOL2CHAR(v));
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
 * Switch between RX and TX
 *---------------------------------------------------------------------------------------------*/
void switch_RXTX(bool t) {  //t=False (RX) : t=True (TX)

#ifdef DEBUG
  if (t != getWord(SSW,TXON)) {
      _EXCPLIST("%s (%s)\n",__func__,BOOL2CHAR(t));
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
     uint32_t freqtx=freq;
     #ifdef CW
        if (mode==MAXMODE-1) {
           freqtx=freq+uint32_t(cwshift);
        } else {
           freqtx=freq;
        }
        #ifdef DEBUG     
          _INFOLIST("%s TX+ (CW On) ftx=%ld f=%ld\n",__func__,freqtx,freq);
        #endif //DEBUG
     #else
        freqtx=freq;
        #ifdef DEBUG     
        _INFOLIST("%s TX+ f=%ld\n",__func__,freqtx);
        #endif //DEBUG
     #endif //CW
         
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
 
}
/*----------------------------------------------------------*
 * Manually turn TX while pressed                           *
 *----------------------------------------------------------*/
void ManualTX(){
   
    switch_RXTX(HIGH);
      
    while(detectKey(TXSW,LOW,false)==LOW) {
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
    }
    switch_RXTX(LOW);
    
    #ifdef ANTIVOX
      tavox=millis();
      setWord(&TSW,AVOX,true);
    #endif //ANTIVOX   
}
/*==================================================================================================*
 * Clock (Si5351) Calibration methods                                                               *
 * Legacy method (ADX)                                                                              *
 *     Clock (CLK2) is set to 1MHz output , calibration factor is increased (UP) or decreased (DOWN)*
 *     until a frequency counter shows 1 MHz, this way any offset on the clock will be compensated  *
 *     calibration factor will be stored in EEPROM and saved till next calibration                  *
 *==================================================================================================*/
/*----------------------------------------------------------*
 * Calibration function (LEGACY, Manual)
 *----------------------------------------------------------*/
void Calibration(){
  
  #ifdef DEBUG
     _EXCP;
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
     _EXCPLIST("%s cal_factor=%ld\n",__func__,cal_factor);
#endif //DEBUG
  
  while (true) {
    
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
  
   setWord(&SSW,SAVEEE,false);

#ifdef DEBUG
   _EXCPLIST("%s save(%d) cal(%d) m(%d) slot(%d) save=%d build=%d\n",__func__,save,cal_factor,mode,Band_slot,save,build);
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
   
   updateEEPROM();
}
/*------
 * checkEEPROM
 * check if there is a pending EEPROM save that needs to be committed
 */
void checkEEPROM() {
    
    if((millis()-tout)>eeprom_tout && getWord(SSW,SAVEEE)==true ) {
       #ifdef DEBUG
          _EXCPLIST("%s() Saving EEPROM...\n",__func__);
       #endif //DEBUG 
      
       updateEEPROM();
    }
 
}

#endif //EE
/**********************************************************************************************/
/*                               Operational state management                                 */
/**********************************************************************************************/
/*----------------------------------------------------------*
 * Band increase                                            *
 *----------------------------------------------------------*/
uint16_t changeBand(uint16_t c) {
    uint16_t b=(Band_slot+c)%BANDS;
    #ifdef DEBUG
       _EXCPLIST("%s() change=%d Band_slot=%d b=%d\n",__func__,c,Band_slot,b);
    #endif //DEBUG 
    return b;
}
/*----------------------------------------------------------*
 * Mode assign                                              *
 *----------------------------------------------------------*/
void Mode_assign(){

   freq=f[mode];
   #ifdef CW
      if (mode==MAXMODE-1) {
        resetLED();
        setLED(JS8,false);
        setLED(FT4,false);
      } else {
        setLED(LED[mode],true);
      }
   #else
      setLED(LED[mode],true);
   #endif //CW
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
      _EXCPLIST("%s mode(%d) f(%ld)\n",__func__,mode,f[mode]);
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
         case  12 : {s=7;break;}
         case  10 : {s=8;break;}
      }
      #ifdef DEBUG
       _EXCPLIST("%s() band=%d slot=%d\n",__func__,b,s);
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
        _EXCPLIST("%s Band=%d slot=%d quad=%d f=%ld\n",__func__,Band,b,q,freq);   
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
       _EXCPLIST("%s B(%d) b[%d] m[%d] slot[%d] f[0]=%ld f[1]=%ld f[2]=%ld f[3]=%ld f=%ld\n",__func__,Band,b,mode,Band_slot,f[0],f[1],f[2],f[3],freq);
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
       _EXCPLIST("%s mode(%d) slot(%d) f=%ld\n",__func__,mode,Band_slot,freq);
    #endif //DEBUG   
  
}
/*----------------------------------------------------------*
 * Select band to operate
 *----------------------------------------------------------*/
void Band_Select(){
  
   resetLED();

   #ifdef DEBUG
      _EXCPLIST("%s slot(%d) LED(%d)\n",__func__,Band_slot,LED[3-Band_slot]);
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
         #ifdef DEBUG
         _EXCPLIST("%s blink TX led\n",__func__);
         #endif //DEBUG
         (l==false ? setLED(TX,false) : clearLED(TX));    
         l=!l;
      }
                
      if (detectKey(UP,LOW,NOWAIT)==LOW) {
          #ifdef DEBUG
          _EXCPLIST("%s Key UP detected\n",__func__);
          #endif //DEBUG
          
          while (detectKey(UP,LOW,WAIT)==LOW){}        
          
          #ifdef DEBUG
          _EXCPLIST("%s Key UP released\n",__func__);
          #endif //DEBUG
          
          Band_slot=changeBand(-1);
          setLED(LED[3-Band_slot],true);

          #ifdef DEBUG
             _EXCPLIST("%s slot(%d)\n",__func__,Band_slot);
          #endif //DEBUG   
      } 
   
      if (detectKey(DOWN,LOW,WAIT)==LOW) {

          #ifdef DEBUG
          _EXCPLIST("%s Key DOWN detected\n",__func__);
          #endif //DEBUG

         while (detectKey(DOWN,LOW,WAIT)==LOW){}

         #ifdef DEBUG
         _EXCPLIST("%s Key DOWN released\n",__func__);
         #endif //DEBUG

         Band_slot=changeBand(+1);
         setLED(LED[3-Band_slot],true);

         #ifdef DEBUG
            _EXCPLIST("%s slot(%d)\n",__func__,Band_slot);
         #endif //DEBUG   

      }                                               
      if (detectKey(TXSW,LOW,NOWAIT) == LOW) {
        
          #ifdef DEBUG
          _EXCPLIST("%s Key TX detected\n",__func__);
          #endif //DEBUG

         while (detectKey(TXSW,LOW,WAIT)==LOW){}

          #ifdef DEBUG
          _EXCPLIST("%s Key TX released\n",__func__);
          #endif //DEBUG

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

/*---------------------------------------------------------------------------*
 *  checkMode
 *  manage change in mode during run-time
 *---------------------------------------------------------------------------*/
void checkMode() {
/*--------------------------------*
 * TX button short press          *
 * Transmit mode                  *
 *--------------------------------*/
    if ((detectKey(TXSW,LOW,NOWAIT) == LOW) && (getWord(SSW,TXON)==false)) {
      
       #ifdef DEBUG
          _EXCPLIST("%s TX+\n",__func__);
       #endif //DEBUG
        
       ManualTX(); 
     
       #ifdef DEBUG
         _EXCPLIST("%s TX-\n",__func__);
       #endif //DEBUG   
  }

/*-------------------------------------------------------------*
 * manage band, mode and  calibration selections               *
 *-------------------------------------------------------------*/
#ifndef ONEBAND
 
  if ((detectKey(UP,LOW,NOWAIT) == LOW)&&(detectKey(DOWN,LOW,NOWAIT) == LOW)&&(getWord(SSW,TXON)==false)) {
     while (detectKey(UP,LOW,NOWAIT) == LOW) {}
     while (detectKey(DOWN,LOW,NOWAIT)==LOW) {} //Wait till both switches are up
     
     Band_Select();
     
     #ifdef DEBUG
      _EXCPLIST("%s U+D f=%ld",__func__,freq);
     #endif //DEBUG 
  }
  
#endif //ONEBAND

  if ((detectKey(DOWN,LOW,NOWAIT) == LOW) && (getWord(SSW,TXON)==false)) {
      while (detectKey(DOWN,LOW,NOWAIT) == LOW) {}
      
      if (mode==0) {
        #ifdef CW
           mode=MAXMODE-1;
        #else   
           mode=MAXMODE-2;
        #endif //CW   
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
   
  if ((detectKey(UP,LOW,NOWAIT) == LOW) && (getWord(SSW,TXON)==false)) {
      while (detectKey(UP,LOW,NOWAIT)==LOW) {}
      mode++;
      /*---
       * CW enables a 5th mode
       */
      #ifdef CW
         if (mode==MAXMODE-1) {
            setWord(&SSW,CWMODE,true);
         } else {
            setWord(&SSW,CWMODE,false);   
         }
         if (mode>MAXMODE-1) {
            mode=0;
         }
      #else
         if (mode>MAXMODE-2) {
            mode=0;
         }
         setWord(&SSW,CWMODE,false);
      #endif //CW   

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

 #ifdef DEBUG
    _INFOLIST("%s EEPROM retrieved temp(%d) & Build(%d) BUILD=%d\n",__func__,temp,build,uint16_t(BUILD));
 #endif //DEBUG
 
 
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
  delay(100);

#ifdef DEBUG
   _INFOLIST("%s setup m(%d) slot(%d) f(%ld)\n",__func__,mode,Band_slot,freq);
#endif //DEBUG      
}
/*--------------------------------------------------------------------------*
 * definePinOut
 * isolate pin definition on a board conditional procedure out of the main
 * setup flow
 *--------------------------------------------------------------------------*/
void definePinOut() {

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
   _EXCP;
#endif //DEBUG   

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

   #if (defined(DEBUG) || defined(CAT))   
      Serial.begin(BAUD,SERIAL_8N2);
      while (!Serial) {
      #ifdef WDT      
         wdt_reset();
      #endif //WDT              
      }
      delay(SERIAL_WAIT);
      Serial.flush();
      Serial.setTimeout(SERIAL_TOUT);    
   #endif //DEBUG or CAT

   #ifdef DEBUG
      const char * proc = "ATmega328P";
      _INFOLIST("%s: ADX Firmware V(%s) build(%d) board(%s)\n",__func__,VERSION,BUILD,proc);
   #endif //DEBUG

/*---
 * List firmware properties at run time
 */

   definePinOut();
   blinkLED(TX);   
   setup_si5351();   
   
   #ifdef DEBUG
      _EXCPLIST("%s setup_si5351 ok\n",__func__);
   #endif //DEBUG   
   
   initADX();
   #ifdef DEBUG
      _EXCPLIST("%s initADX ok\n",__func__);
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
   if (detectKey(DOWN,LOW,WAIT)==LOW) { 
      #ifdef DEBUG
        _EXCPLIST("%s Calibration mode detected\n",__func__);
      #endif //DEBUG
      Calibration();
   }
  
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
     _EXCPLIST("%s Counting algorithm TIMER1 set Ok\n",__func__);
  #endif //DEBUG   

  
  switch_RXTX(LOW);
  #ifdef DEBUG
      _EXCPLIST("%s switch_RXTX Low ok\n",__func__);
  #endif //DEBUG   

  Mode_assign(); 

  #ifdef WDT
 
     wdt_disable();
     wdt_enable(WDTO_8S);    
     setWord(&TSW,TX_WDT,false);
  #endif //WDT

  #ifdef DEBUG
     _INFOLIST("%s completed\n",__func__);
  #endif //DEBUG   
 
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
//*--- if CAT enabled check for serial events (TS480 & IC746)
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
 * main transmission loop       (ADX)                                               *
 * Timer1 (16 bits) with no pre-scaler (16 MHz) is checked to detect zero crossings *
 * if there is no overflow the frequency is calculated                              *
 * if activity is detected the TX is turned on                                      *
 * TX mode remains till no further activity is detected (operate like a VOX command)*
 *----------------------------------------------------------------------------------*/
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
