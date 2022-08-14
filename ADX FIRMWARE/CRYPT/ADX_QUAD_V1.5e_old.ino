
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
//* Modified by Dr. P.E.Colla (LU7DZ)                                                                               
//*     X re-style of the code to facilitate customization for multiple boards
//*     X Add all frequency definitions for HF bands
//*     X Optimize EEPROM read/write cycles
//*     X add CAT support (TS-440), thru FLRig (see README.md)
//*     X add timeout & watchdog support (both hardware glitches and extended PTT time)
//*     X support for the QUAD/OCTO band filter boards
//*     X support for an external ATU (D5 line)
//*     X support for the ICOM-746 CAT Protocol
//*     x serial configuration tool
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
/*-------------------------------------------------------------*
 * Define the runtime platform either PICO (Raspberry Pi Pico) *
 * or !PICO (Arduino ATMega328p)                               *
 *-------------------------------------------------------------*/
//#define ADX              1   //This is the standard ADX Arduino based board 
#define PDX            1   //Compile for Raspberry Pi Pico board

#ifdef PDX
   #pragma GCC optimize (0)
#endif //PDX
 
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                            EXTERNAL LIBRARIES USED                                          *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*

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
   #include "hardware/pwm.h"
   #include "pico/multicore.h"  
#endif //PDX
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                            VERSION HEADER                                                   *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
#define VERSION        "1.5e"
#define BUILD          135

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
//*                            (A)rduino (D)igital (X)ceiver                                    *
//*                            FEATURE CONFIGURATION PROPERTIES                                 *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*

#ifdef ADX
   #define WDT            1      //Hardware and TX watchdog enabled
   #define EE             1      //User EEPROM for persistence
   #define CAT            1      //Enable CAT protocol over serial port
   #define TS480          1      //CAT Protocol is Kenwood 480
   #define QUAD           1      //Enable the usage of the QUAD 4-band filter daughter board
/*
 * The following definitions are disabled but can be enabled selectively
 */
   //#define ONEBAND        1      //Forces a single band operation in order not to mess up because of a wrong final filter
   //#define ATUCTL         1      //Control external ATU device
   //#define RESET          1      //Allow a board reset (*)-><Band Select> -> Press & hold TX button for more than 2 secs will reset the board (EEPROM preserved)
   //#define CW             1      //CW support
   //#define CAL_RESET      1      //If enabled reset cal_factor when performing a new calibration()
   //#define DEBUG          1      //DEBUG turns on different debug, information and trace capabilities, it is nullified when CAT is enabled to avoid conflicts
   //#define TERMINAL       1      //Serial configuration terminal
   //#define FT817          1      //CAT Protocol is FT 817

#endif //PICO

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                               (P)ico (D)igital (X)ceiver                                    *
//*                            FEATURE CONFIGURATION PROPERTIES                                 *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
#ifdef PDX
   #define WDT             1      //Hardware and TX watchdog enabled
   #define EE              1      //Save in Flash emulation of EEPROM the configuration
   //#define CW              1      //CW support
   //#define CAT             1      //Enable CAT protocol over serial port
   //#define FT817           1      //CAT protocol is Yaesu FT817
   //#define ATUCTL          1      //Brief 200 mSec pulse to reset ATU on each band change
   //#define QUAD            1      //Support for QUAD board
   #define ONEBAND         1      //Define a single band 
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
#ifdef ADX
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

#endif //ADX


#ifdef PDX

/*---- 
 * Output control lines
 */
   #define RX              2      //RX Switch
#ifdef ATUCTL
   #define ATU            15     //ATU Device control line (flipped HIGH during 200 mSecs at a band change)
#endif //ATUCTL 

/*---
 * LED
 */   
   #define WSPR            7      //WSPR LED 
   #define JS8             6      //JS8 LED
   #define FT4             5      //FT4 LED
   #define FT8             4      //FT8 LED
   #define TX              3      //TX LED  
/*---
 * Switches
 */
   #define UP             10      //UP Switch (this must be set to GPIO19 when running on a PDX board)
   #define DOWN           11      //DOWN Switch (this must be set to GPIO20 when running on a PDX board) 
   #define TXSW            8      //TX Switch
/*---
 *  I2C
 */
   #define PDX_I2C_SDA    16      //I2C SDA
   #define PDX_I2C_SCL    17      //I2C SCL
/*---
 *  Input lines
 */
   #define FSK            27      //Frequency counter algorithm
   #define CAL             9      //Automatic calibration entry
#endif //PDX

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


#ifdef EE
const char *eeprom_toutToken= "*eet";
const char *eeprom_listToken= "*list";
#endif //EE


const char *saveToken       = "*save"; 
const char *quitToken       = "*quit";
const char *resetToken      = "*reset";
const char *helpToken       = "*help";
const char *endList         = "XXX";    

#endif //TERMINAL

#ifdef TS480
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*               DEFINITIONS SPECIFIC TO TS480 CAT PROTOCOL                                    *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*

   #define CATCMD_SIZE          18
   
   volatile char    CATcmd[CATCMD_SIZE];
   const int        BUFFER_SIZE = CATCMD_SIZE;
   char             buf[BUFFER_SIZE];

#endif //TS480

#ifdef PDX
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*               DEFINITIONS SPECIFIC TO THE RP2040 ARCHITECTURE                               *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
#define  CAL_COMMIT      12
#define  CAL_ERROR        1

//#define  FSK_PEG         1
#define  FSK_ZCD         1

#if defined(FSK_PEG) && defined(FSK_ZCD)
    #undef FSK_PEG
#endif //Consistency rule 

#define FSKMIN             300    //Minimum FSK frequency computed
#define FSKMAX            2800    //Maximum FSK frequency computed

#if FSK_PEG
    #define  FSK_WINDOW      10
    #define  FSK_WINDOW_USEC FSK_WINDOW*1000
    #define  FSK_MULT        1000/FSK_WINDOW
    #define  FSK_IDLE        1000*FSK_WINDOW*2
#endif //FSK_PEG

#ifdef FSK_ZCD
    #define FSK_USEC                  1000000
    #define FSK_SAMPLE                   1000
    #define FSK_IDLE      5*FSK_SAMPLE*FSK_RA
    #define FSK_ERROR                       4
    #define FSK_RA                         20
#endif //FSK_ZCD

uint32_t f_hi;
int      pwm_slice;  
uint32_t ffsk     = 0;
uint32_t fclk     = 0;
int32_t  error    = 0;
uint32_t codefreq = 0;
uint32_t prevfreq = 0;

#endif //PDX
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*               DEBUG SUPPORT MACRO DEFINITIONS                                               *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*

/*****************************************************************
 * Trace and debugging macros (only enabled if DEBUG is set      *
 *****************************************************************/
#define DEBUG  1
#ifdef DEBUG        //Remove comment on the following #define to enable the type of debug macro
   #define INFO  1   //Enable _INFO and _INFOLIST statements
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

   #ifdef TERMINAL
      #define EEPROM_ATU          60
      #define EEPROM_ATU_DELAY    70
      #define EEPROM_BOUNCE_TIME  80
      #define EEPROM_SHORT_TIME   90
      #define EEPROM_MAX_BLINK   120
      #define EEPROM_EEPROM_TOUT 130
      #define EEPROM_AVOXTIME    170
      #define EEPROM_END         200
   #endif //TERMINAL

   uint32_t tout=0;

   //#define EEPROM_CLR     1   //Initialize EEPROM (only to be used to initialize contents)
   #define EEPROM_SAVED  100     //Signature of EEPROM being updated at least once
   #define EEPROM_TOUT  2000     //Timeout in mSecs to wait till commit to EEPROM any change
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
uint8_t  QSW            = 0;
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
      _INFO;
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
   _INFOLIST("%s() f=%ld band=%d slot=%d\n",__func__,f,b,s);
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
      /*---
       * operate band switching if necessary and the
       * selection of the QUAD filter if enabled
       */
    /*   
 int   b=setSlot(uint32_t(f));
 */
 /*
       if (b<0) {
          response[0] = 0;
          Serial.write(response, 1);
          #ifdef ADX
            delay(SERIAL_WAIT);
            Serial.flush();
            delay(50);
          #endif   
          break;
          }
  */
       freq=f;

 /*--- 
  * If a band change is detected switch to the new band
  *---*/
    /*
       if (b!=Band_slot) { //band change
           Band_slot=b;
           Freq_assign();
           freq=f;
        }
    */
  /*---
   * Properly register the mode if the frequency implies a WSJT mode change (FT8,FT4,JS8,WSPR) ||
   */

    
    /*
        int i=getBand(freq);
        if ( i<0 ) {
           break;
        }
        
        int j=findSlot(i);
        if (j<0 || j>3) {
           break;
        }
        int k=Bands[j];
        int q=band2Slot(k);
        int m=getMode(q,freq);
  */
  /*
        #ifdef DEBUG
           _INFOLIST("%s f=%ld band=%d slot=%d Bands=%d b2s=%d m=%d mode=%d\n",__func__,freq,i,j,k,q,m,mode);
        #endif //DEBUG  

        if (getWord(SSW,CWMODE)==false) {   
  
           if (mode != m) {
              mode = m;
              Mode_assign();
           }
        }
   */     
/*----
 * if enabled change filter from the LPF filter bank
 *----*/
    /*
        #ifdef QUAD  //Set the PA & LPF filter board settings if defined
           int x=band2QUAD(k);
           if (x != -1) {
              setQUAD(x);
           }   
        #endif //QUAD    

        #ifdef DEBUG
          _INFOLIST("%s() CAT=%s f=%ld slot=%d bands[]=%d slot=%d quad=%d\n",__func__,Catbuffer,freq,b,k,q,x);
        #endif //DEBUG 
*/
        //freq = uint32_t(f);
        response[0] = 0;
        Serial.write(response, 1);
        #ifdef ADX
           delay(SERIAL_WAIT);
           Serial.flush();
           delay(50);
        #endif   
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
      #ifdef ADX
         delay(SERIAL_WAIT);
         Serial.flush();
         delay(50);
      #endif   

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
      #ifdef ADX
         delay(SERIAL_WAIT);
         Serial.flush();
         delay(50);
      #endif   
      // setFrequency(frequency);
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
      #ifdef ADX
         delay(SERIAL_WAIT);
         Serial.flush();
         delay(50);
      #endif   

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
      #ifdef ADX
         delay(SERIAL_WAIT);
         Serial.flush();
         delay(50);
      #endif   
      break;
    }
    case 0x81: // toggle the VFOs
  {
      response[0] = 0;
      Serial.write(response, 1);
      #ifdef ADX
         delay(SERIAL_WAIT);
         Serial.flush();
         delay(50);
      #endif   

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
      #ifdef ADX
         delay(SERIAL_WAIT);
         Serial.flush();
         delay(50);
      #endif   

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
        #ifdef ADX
           delay(SERIAL_WAIT);
           Serial.flush();
           delay(50);
        #endif   

      }
      break;

    default:
    {
      response[0] = 0x00;
      Serial.write(response[0]);
      #ifdef ADX
         delay(SERIAL_WAIT);
         Serial.flush();
         delay(50);
      #endif   
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
 * needed to interact with WSJT-X (or FLRig)  many answers aren't actually other                       *
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
  const char *cFR="FR"; const char *cFT="FT"; const char *cEX="EX";
  const char *cKY="KY"; const char *cXT="XT"; const char *cVX="VX";
  const char *cRU="RU"; const char *cPS="PS"; const char *cRD="RD"; 

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
  _INFOLIST("%s f=%ld band=%d slot=%d Bands=%d b2s=%d m=%d mode=%d\n",__func__,freq,i,j,k,q,m,mode);
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
      _INFOLIST("%s() CAT=%s f=%ld slot=%d bands[]=%d slot=%d quad=%d\n",__func__,Catbuffer,freq,b,k,q,x);
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
}

//*--- Place transceiver in TX mode
void Command_TX()
{

  switch_RXTX(HIGH);
  setWord(&SSW,CATTX,true);
  sprintf(hi,"%s",cTX0);
  Serial.print(hi);
}

//*--- Response for VOX command
void Command_VX()
{
  sprintf(hi,"VX%s;",(getWord(SSW,VOX)==true ? "1" : "0"));
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
//*--- Fake response for AS; command (not implemented)
void Command_AS() {

  sprintf(hi,"AS000%011ld%c;",freq,modeTS480());
  Serial.print(hi);
  return;
}
//*--- Fake response for XI; command (not implemented)
void Command_XI() {

  sprintf(hi,"XI%011ld%c0;",freq,modeTS480());
  Serial.print(hi);
  return;
}

//*--- Band change command (not implemented)
void Command_BChange(int c) { //Change band up or down

  Band_slot=changeBand(c);
  Band_assign();
  #ifdef DEBUG
      _TRACELIST("%s() change=%d Band_slot=%d\n",__func__,c,Band_slot);
  #endif //DEBUG 
  
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
  if ((CATcmd[0] == 'B') && (CATcmd[1] == 'D'))                        {Command_BChange(-1); return;}
  if ((CATcmd[0] == 'B') && (CATcmd[1] == 'U'))                        {Command_BChange(+1); return;}

  
  strcmd[0]=CATcmd[0];
  strcmd[1]=CATcmd[1];
  strcmd[2]=0x00;

  if (strcmp(strcmd,cID)==0)                                                      {sprintf(hi,"%s",cIDr);Serial.print(hi);return;}
  if (strcmp(strcmd,cFR)==0 || strcmp(strcmd,cFT)==0 || strcmp(strcmd,cEX)==0 ||
      strcmp(strcmd,cVX)==0 || strcmp(strcmd,cXT)==0 || strcmp(strcmd,cKY)==0 ||
      strcmp(strcmd,cRU)==0 || strcmp(strcmd,cPS)==0 || strcmp(strcmd,cRD)==0)    {sprintf(hi,"%s",CATcmd);Serial.print(hi);return;}

             
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

#ifdef PDX
  if (LEDpin==uint8_t(TX)) {setGPIO(LED_BUILTIN,LOW);}
#endif //PDX    

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
   
#ifdef PDX   
   clearLED(LED_BUILTIN);
#endif //PDX   

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

#ifdef PDX
  if (LEDpin==uint8_t(TX)) {setGPIO(LED_BUILTIN,HIGH);}
#endif //PDX    

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
       
       #ifdef PDX
         if (LEDpin==uint8_t(TX)) {setGPIO(LED_BUILTIN,HIGH);}
       #endif //PDX    

       delay(BDLY);
       setGPIO(LEDpin,LOW);
       #ifdef PDX
         if (LEDpin==uint8_t(TX)) {setGPIO(LED_BUILTIN,LOW);}
       #endif //PDX    

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
   _INFO;
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
         _INFOLIST("%s check pin(%d)\n",__func__,p);
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
            _INFOLIST("%s pin(%d) [%d]->[%d]\n",__func__,p,getWord(button[p],PUSHSTATE),pstate);
         #endif //DEBUG         
         
         setWord(&button[p],PUSHSTATE,pstate);
         if (pstate == LOW) {
           downTimer[p]=millis();
         } else {
           timerDown=millis()-downTimer[p];
           if (timerDown<bounce_time) {
              #ifdef DEBUG
                 _INFOLIST("%s pin(%d) too short, ignored!\n",__func__,p);
              #endif //DEBUG   
              downTimer[p]=millis();  //fix weird Barb pushbutton with a 2nd train of bouncing signals
            
           }
           setWord(&SSW,v,true);
           if (timerDown<short_time){
              setWord(&button[p],SHORTPUSH,true);
              setWord(&button[p],LONGPUSH,false);
              
              #ifdef DEBUG
                 _INFOLIST("%s pin(%d) <SP>\n",__func__,p);
              #endif //DEBUG

           } else {
              setWord(&button[p],SHORTPUSH,false);
              setWord(&button[p],LONGPUSH,true);       
              #ifdef DEBUG
                 _INFOLIST("%s pin(%d) <LP>\n",__func__,p);
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
                 #ifdef DEBUG
                   _INFOLIST("%s switch(%d) value(%s)\n",__func__,k,BOOL2CHAR(v));
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
        _INFOLIST("%s TX+ (CW=%s) TX=%s ftx=%ld f=%ld\n",__func__,BOOL2CHAR(getWord(SSW,CWMODE)),BOOL2CHAR(getWord(SSW,TXON)),freqtx,freq);
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

#ifdef PDX
     setGPIO(LED_BUILTIN,HIGH);
#endif //PDX
          
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
       _INFOLIST("%s RX+ f=%ld\n",__func__,freq);
    }
#endif //DEBUG
    
    si5351.set_freq(freq*100ULL, SI5351_CLK1);
    si5351.output_enable(SI5351_CLK1, 1);   //RX on
    
    setGPIO(TX,0); 

#ifdef PDX
    setGPIO(LED_BUILTIN,LOW);
#endif //PDX    

    setWord(&SSW,TXON,LOW);
    setWord(&SSW,VOX,LOW);
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
    
    #ifdef DEBUG
       _INFOLIST("%s ManualTX(HIGH)\n",__func__);
    #endif //DEBUG   
    
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
    #ifdef DEBUG
       _INFOLIST("%s ManualTX(LOW)\n",__func__);
    #endif //DEBUG   
    
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
          _INFOLIST("%s (%d): <SP>\n",__func__,p);
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
/*==================================================================================================*
 * Clock (Si5351) Calibration methods                                                               *
 * Legacy method (ADX)                                                                              *
 *     Clock (CLK2) is set to 1MHz output , calibration factor is increased (UP) or decreased (DOWN)*
 *     until a frequency counter shows 1 MHz, this way any offset on the clock will be compensated  *
 *     calibration factor will be stored in EEPROM and saved till next calibration                  *
 *  Automatic method (PDX)                                                                          *
 *     Clock (CLK2) is set to 10MHz output, the board connects this value to the GPIO8 (CAL) pin.   *
 *     An iteration is made automatically until the read value is 10MHz.                            *
 *     The calibration factor will be store                                                         *
 *==================================================================================================*/
#if defined(ADX)
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*                                     ADX Calibration procedure (legacy,manual)                           *
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
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
#endif //Legacy calibration method (ADX)


#if defined(PDX)

//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*                                     PDX Calibration procedure (automatic)                               *
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
/*------
 * Înterrupt IRQ for edge counting overflow
 *-----*/
void pwm_int() {
   pwm_clear_irq(pwm_slice);
   f_hi++;
}

/*=========================================================================================*
 * CORE1                                                                                   *
 * 2nd rp2040 core instantiated by defining setup1/proc1 procedures                        *
 * These procedures are used to run frequency measurement / time sensitive code            *
 * the por1 procedure isn't never reached actually as the flow is left at an infinite loop *
 * at setup1                                                                               *
 *=========================================================================================*/
void setup1() {

/*-----------------------------------------------------------------*
 * Core1   Setup procedure                                         *
 * Enter processing on POR but restarted from core0 setup ()       *
 *-----------------------------------------------------------------*/
uint32_t t = 0;
bool     b = false;
 /*--------------------------------------------*
  * Wait for overall initialization to complete*
  *--------------------------------------------*/
  while (getWord(QSW,QWAIT)==false) {
    
    #ifdef WDT
       wdt_reset();
    #endif //WDT
    
    uint32_t t = time_us_32() + 2;
    while (t > time_us_32());
  }
  /*-------------------------------------------*
   * Semaphore QWAIT has been cleared, proceed *
   * PWM counters operates as infinite loops   *
   * therefore no loop1() is ever processed    *
   *-------------------------------------------*/
   #ifdef DEBUG
      _INFOLIST("%s Core1 waiting semaphore released QCAL=%s QFSK=%s\n",__func__,BOOL2CHAR(QCAL),BOOL2CHAR(QFSK));
   #endif //DEBUG
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//* Automatic calibration procedure                                                                             *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*   
   if (getWord(QSW,QCAL)==true) {
      #ifdef DEBUG
         _INFOLIST("%s Calibration procedure triggered\n",__func__);
      #endif //DEBUG    
      delay(1000);
      calibrateLED();
      
      /*----
       * Prepare Si5351 CLK2 for calibration process
       *---*/
       #ifdef DEBUG
         _INFOLIST("%s Automatic calibration procedure started\n",__func__);
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
       * PWM counter used for automatic calibration *
       * -------------------------------------------*/
      fclk=0;
      int16_t n=int16_t(CAL_COMMIT);
      cal_factor=0;
      #ifdef DEBUG
        _INFOLIST("%s si5351 initialization ok target freq=%ld cal_factor=%ld\n",__func__,Cal_freq,cal_factor);
      #endif //DEBUG     
      
      pwm_slice=pwm_gpio_to_slice_num(CAL);      
      while (true) {
          /*-------------------------*
           * setup PWM counter       *
           *-------------------------*/
          pwm_config cfg=pwm_get_default_config();
          pwm_config_set_clkdiv_mode(&cfg,PWM_DIV_B_RISING);
          pwm_init(pwm_slice,&cfg,false);
          gpio_set_function(CAL,GPIO_FUNC_PWM);
          
          pwm_set_irq_enabled(pwm_slice,true);
          irq_set_exclusive_handler(PWM_IRQ_WRAP,pwm_int);
          irq_set_enabled(PWM_IRQ_WRAP,true);
          f_hi=0;

          /*---------------------------*
           * PWM counted during 1 sec  *
           *---------------------------*/
          t=time_us_32()+2;
          while (t>time_us_32());
          pwm_set_enabled(pwm_slice,true);         
          t+=1000000;
          while (t>time_us_32());
          pwm_set_enabled(pwm_slice,false);

          /*----------------------------*
           * recover frequency in Hz    *
           *----------------------------*/
          fclk=pwm_get_counter(pwm_slice);
          fclk+=f_hi<<16;
          error=fclk-Cal_freq;
          #ifdef DEBUG
            _INFOLIST("%s Calibration VFO=%ld Hz target_freq=%ld error=%ld cal_factor=%ld\n",__func__,fclk,Cal_freq,error,cal_factor);
          #endif //DEBUG            
          if (labs(error) > int32_t(CAL_ERROR)) {          
             b=!b;
             if (b) {
                setLED(TX,false);
             } else {
                rstLED(TX,false);               
             }
             if (error < 0) {
                cal_factor=cal_factor - CAL_STEP;
             } else {
                cal_factor=cal_factor + CAL_STEP;
             }
             si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
          } else {
            n--;
            if (n==0) {
               #ifdef DEBUG
                 _INFOLIST("%s Convergence achieved cal_factor=%ld\n",__func__,cal_factor);
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
                 setLED(JS8,true);
                 setLED(FT4,false);
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
   if (getWord(QSW,QFSK)==true) {

      ffsk=0;
      uint16_t cnt=100;
      pwm_slice=pwm_gpio_to_slice_num(FSK);

      #ifdef DEBUG
         _INFOLIST("%s FSK counter() triggered\n",__func__);
      #endif //DEBUG    

/*--------------------------------------------------------------*      
 * main counting algorithm cycle                                *
 *--------------------------------------------------------------*/
      while (true) {
          pwm_config cfg=pwm_get_default_config();
          pwm_config_set_clkdiv_mode(&cfg,PWM_DIV_B_RISING);
          pwm_init(pwm_slice,&cfg,false);
          gpio_set_function(FSK,GPIO_FUNC_PWM);
          pwm_set_irq_enabled(pwm_slice,true);
          irq_set_exclusive_handler(PWM_IRQ_WRAP,pwm_int);
          irq_set_enabled(PWM_IRQ_WRAP,true);
          f_hi=0;
          /*---------------------------------------*
           * PEG algorithm                         *
           * defined by FSK_PEG                    *
           * this is based on a pure PWM counting  *
           * over a defined window for counting    *
           * it has a +/- 1 count error which      *
           * translate into a FSK_MULT (Hz) of     *
           * counting error                        *
           * FSK_MULT must be larger compared with *
           * actual bandwidth of a given signal    *
           * to reduce counting error impact.      *
           * Some heuristics might apply to correct*
           * this error.                           *
           *---------------------------------------*/
          #ifdef FSK_PEG
             uint32_t t=time_us_32()+2;                         //Wait 2 uSec for definitions to stabilize
             while (t>time_us_32());                            //
             pwm_set_enabled(pwm_slice,true);                   //Enable pwm count
             t+=uint32_t(FSK_WINDOW_USEC);                      //Wait for the FSK WINDOW (uSec)
             while (t>time_us_32());                            //This window will define the sample rate for frequency (every FSK_WINDOW uSec)
             pwm_set_enabled(pwm_slice,false);                  //Disable pwm count
             ffsk=pwm_get_counter(pwm_slice);                   //Obtain actual pwm count during the window
             ffsk+=f_hi<<16;                                    //Add overflow if any
             ffsk=ffsk*FSK_MULT;                                //Apply window multiplicator (1000/FSK_WINDOW)
             _INFOLIST("%s f_hi=%ld ffsk=%ld\n",__func__,f_hi,ffsk);
             if (ffsk > FSKMIN && ffsk <= FSKMAX) {             //If frequency is outside the allowed bandwidth ignore
                 rp2040.fifo.push(ffsk);                         //Use the rp2040 FIFO IPC to communciate the new frequency
             }
          #endif //FSK_PEG algorithm

          #ifdef FSK_ZCD
         /*----------------------------------------*
           * ZCD algorithm                         *
           * defined by FSK_ZCD                    *
           * this is based on a pseudo cross detect*
           * where the rising edge is taken as a   *
           * false cross detection followed by next*
           * edge which is also a false zcd but    *
           * at the same level thus measuring the  *
           * time between both will yield a period *
           * measurement proportional to the real  *
           * period of the signal as measured      *
           * two sucessive rising edges            *
           * Measurements are made every 1 mSec    *
           *---------------------------------------*/             
             uint32_t t=time_us_32()+2;                         //Allow all the settings to stabilize
             while (t>time_us_32());                            //
             uint16_t j=FSK_RA;                                 //
             uint32_t dt=0;                                      //
             while (j>0) {                                      //Establish a running average over <j> counts
                uint32_t pwm_cnt=pwm_get_counter(pwm_slice);    //Get current pwm count
                pwm_set_enabled(pwm_slice,true);                //enable pwm count
                while (pwm_get_counter(pwm_slice) == pwm_cnt){} //Wait till the count change
                pwm_cnt=pwm_get_counter(pwm_slice);             //Measure that value
                uint32_t t1=time_us_32();                       //Mark first tick (t1)
                while (pwm_get_counter(pwm_slice) == pwm_cnt){} //Wait till the count change (a rising edge)
                uint32_t t2=time_us_32();                       //Mark the second tick (t2)
                pwm_set_enabled(pwm_slice,false);               //Disable counting
                dt=dt+(t2-t1);                                  //Add to the RA total
                j--;                                            //Loop
             }                                                  //
             if (dt != 0) {                                     //Prevent noise to trigger a nul measurement
                double dx=1.0*dt/double(FSK_RA);                //
                double f=double(FSK_USEC)/dx;                   //Ticks are expressed in uSecs so convert to Hz
                double f1=round(f);                             //Round to the nearest integer 
                ffsk=uint32_t(f1);                              //Convert to long integer for actual usage
                if (ffsk >= FSKMIN && ffsk <= FSKMAX) {         //Only yield a value if within the baseband 
                    rp2040.fifo.push_nb(ffsk);                   //Use the rp2040 FIFO IPC to communicate the new frequency
                   #ifdef DEBUG
                       _TRACELIST("%s dt=%ld dx=%.3f f=%.3f f1=%.3f ffsk=%ld\n",__func__,dt,dx,f,f1,ffsk); 
                   #endif //DEBUG
                }                                               //
             }                                                  //
             t=time_us_32()+FSK_SAMPLE;                         //Now wait for 1 mSec till next sample
             while (t>time_us_32()) ;
          #endif //FSK_ZCD
            
        }  //end FSK loop  
   }
}
#endif //Auto Calibration & Detection algorithm running on Core1
/*==========================================================================================================*/
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

   
#endif //TERMINAL

#ifdef PDX
   EEPROM.commit();
   #ifdef DEBUG
      _INFOLIST("%s commit()\n",__func__)
   #endif //DEBUG
#endif //PDX
   
   setWord(&SSW,SAVEEE,false);

#ifdef DEBUG
   _INFOLIST("%s save(%d) cal(%d) m(%d) slot(%d) save=%d build=%d\n",__func__,save,cal_factor,mode,Band_slot,save,build)
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
   //* Retain calibration cal_factor=0;

#ifdef TERMINAL

#ifdef ATUCTL
   atu        = ATU;
   atu_delay  = ATU_DELAY;
#endif //ATUCTL
   
   bounce_time= BOUNCE_TIME;
   short_time = SHORT_TIME;
   max_blink  = MAX_BLINK;
   eeprom_tout= EEPROM_TOUT;

#endif //TERMINAL

   updateEEPROM();
}
/*------
 * checkEEPROM
 * check if there is a pending EEPROM save that needs to be committed
 */
void checkEEPROM() {
    
    if((millis()-tout)>eeprom_tout && getWord(SSW,SAVEEE)==true ) {
       #ifdef DEBUG
          _INFOLIST("%s() Saving EEPROM...\n",__func__);
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
       _INFOLIST("%s() change=%d Band_slot=%d b=%d\n",__func__,c,Band_slot,b);
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
         case  12 : {s=7;break;}
         case  10 : {s=8;break;}
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
         #ifdef DEBUG
         _EXCPLIST("%s blink TX led\n",__func__);
         #endif //DEBUG
         (l==false ? setLED(TX,false) : clearLED(TX));    
         l=!l;
      }
                
      if (detectKey(UP,LOW,NOWAIT)==LOW) {
          #ifdef DEBUG
          _INFOLIST("%s Key UP detected\n",__func__);
          #endif //DEBUG
          
          while (detectKey(UP,LOW,WAIT)==LOW){}        
          
          #ifdef DEBUG
          _INFOLIST("%s Key UP released\n",__func__);
          #endif //DEBUG
          
          Band_slot=changeBand(-1);
          setLED(LED[3-Band_slot],true);

          #ifdef DEBUG
             _INFOLIST("%s slot(%d)\n",__func__,Band_slot);
          #endif //DEBUG   
      } 
   
      if (detectKey(DOWN,LOW,WAIT)==LOW) {

          #ifdef DEBUG
          _INFOLIST("%s Key DOWN detected\n",__func__);
          #endif //DEBUG

         while (detectKey(DOWN,LOW,WAIT)==LOW){}

         #ifdef DEBUG
         _INFOLIST("%s Key DOWN released\n",__func__);
         #endif //DEBUG

         Band_slot=changeBand(+1);
         setLED(LED[3-Band_slot],true);

         #ifdef DEBUG
            _INFOLIST("%s slot(%d)\n",__func__,Band_slot);
         #endif //DEBUG   

      }                                               
      if (detectKey(TXSW,LOW,NOWAIT) == LOW) {
        
          #ifdef DEBUG
          _INFOLIST("%s Key TX detected\n",__func__);
          #endif //DEBUG

         while (detectKey(TXSW,LOW,WAIT)==LOW){}

          #ifdef DEBUG
          _INFOLIST("%s Key TX released\n",__func__);
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
          _INFOLIST("%s TX+\n",__func__);
       #endif //DEBUG
        
       ManualTX(); 
     
       #ifdef DEBUG
         _INFOLIST("%s TX-\n",__func__);
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
      _INFOLIST("%s U+D f=%ld",__func__,freq);
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
         _INFOLIST("%s m+(%d)\n",__func__,mode);
      #endif //DEBUG
      
      #ifdef EE
         EEPROM.put(EEPROM_MODE, mode); 
      #endif //EEPROM     

      Mode_assign();

      #ifdef DEBUG
         _INFOLIST("%s mode assigned(%d)\n",__func__,mode);
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
         _INFOLIST("%s m-(%d)\n",__func__,mode);
      #endif //DEBUG   

      Mode_assign();

      #ifdef DEBUG
         _INFOLIST("%s mode assigned(%d)\n",__func__,mode);
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

#ifdef TERMINAL

#ifdef ATUCTL
   EEPROM.get(EEPROM_ATU,atu);
   EEPROM.get(EEPROM_ATU_DELAY,atu_delay);
#endif //ATUCTL
   
   EEPROM.get(EEPROM_BOUNCE_TIME,bounce_time);
   EEPROM.get(EEPROM_SHORT_TIME,short_time);
   EEPROM.get(EEPROM_MAX_BLINK,max_blink);
   EEPROM.get(EEPROM_EEPROM_TOUT,eeprom_tout);
   
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
   gpio_set_dir(DOWN,GPIO_IN);
   gpio_set_dir(TXSW,GPIO_IN);

   gpio_pull_up(TXSW);
   gpio_pull_up(DOWN);
   gpio_pull_up(UP);
   
   gpio_set_dir(RX,GPIO_OUT);
   gpio_set_dir(TX, GPIO_OUT);
   gpio_set_dir(LED_BUILTIN, GPIO_OUT);
   gpio_set_dir(WSPR,GPIO_OUT);
   gpio_set_dir(JS8,GPIO_OUT);
   gpio_set_dir(FT4,GPIO_OUT);
   gpio_set_dir(FT8,GPIO_OUT);
   
   gpio_set_dir(FSK,GPIO_IN);

#ifdef ATUCTL
   gpio_init(uint8_t(atu));
   gpio_set_dir (uint8_t(atu), GPIO_OUT);
   flipATU();
#endif //ATUCTL      


   Wire.setSDA(PDX_I2C_SDA);
   Wire.setSCL(PDX_I2C_SCL);
   Wire.begin();



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

   #if (defined(DEBUG) || defined(CAT) || defined(TERMINAL) )   
      Serial.begin(BAUD,SERIAL_8N2);
      while (!Serial) {
      #ifdef WDT      
         wdt_reset();
      #endif //WDT              
      }
      delay(SERIAL_WAIT);
      Serial.flush();
      Serial.setTimeout(SERIAL_TOUT);    
   #endif //DEBUG or CAT or Terminal

   #ifdef DEBUG
      #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
          const char * proc = "ATmega328P";
      #else
          const char * proc = "RP2040";     
      #endif         
      _INFOLIST("%s: ADX Firmware V(%s) build(%d) board(%s)\n",__func__,VERSION,BUILD,proc);
    
      #ifdef DEBUG
          #ifdef TS480
             _INFOLIST("%s: CAT subsystem TS480\n",__func__);
          #endif
          #ifdef IC746
             _INFOLIST("%s: CAT subsystem IC746\n",__func__);
          #endif
          #ifdef FT817
             _INFOLIST("%s: CAT subsystem FT817\n",__func__);
          #endif
      #endif //DEBUG    

   #endif //DEBUG

   #ifdef PDX
      EEPROM.begin(512);
      #ifdef DEBUG
         _INFOLIST("%s: EEPROM reserved (%d)\n",__func__,EEPROM.length());     
      #endif //DEBUG
   #endif //PDX

/*---
 * List firmware properties at run time
 */
#ifdef PDX
   #ifdef DEBUG
      #ifdef EE
      _INFOLIST("%s EEPROM Sub-system activated\n",__func__);
      #endif //EE
      
      #ifdef WDT 
      _INFOLIST("%s Watchdog Sub-system activated\n",__func__);
      #endif //WDT
      
      #ifdef TERMINAL
      _INFOLIST("%s Terminal Sub-system activated\n",__func__);
      #endif //TERMINAL

      #ifdef RESET
      _INFOLIST("%s Reset feature activated\n",__func__);
      #endif //TERMINAL

      #ifdef ATUCTL
      _INFOLIST("%s ATU Reset Sub-system activated\n",__func__);
      #endif //ATUCTL

      #ifdef ONEBAND
      _INFOLIST("%s ONE BAND feature activated\n",__func__);
      #else
      _INFOLIST("%s MULTI BAND feature activated\n",__func__);
      #endif //ONEBAND

      #ifdef QUAD
      _INFOLIST("%s Quad Band filter support activated\n",__func__);
      #endif //ONEBAND

      #ifdef FSK_PEG
      _INFOLIST("%s PEG decoding algorithm used Mult(%d) Window[uSec]=%d \n",__func__,uint16_t(FSK_MULT),uint16_t(FSK_WINDOW_USEC));
      #endif //ONEBAND

      #ifdef FSK_ZCD
      _INFOLIST("%s ZCD decoding algorithm used\n",__func__);
      #endif //ONEBAND

   #endif //DEBUG
#endif //PDX   

   definePinOut();
   blinkLED(TX);   
   setup_si5351();   
   
   #ifdef DEBUG
      _INFOLIST("%s setup_si5351 ok\n",__func__);
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
      _INFOLIST("%s initADX ok\n",__func__);
   #endif //DEBUG   
   
   #ifdef QUAD
     setupQUAD();
     #ifdef DEBUG
        _INFOLIST("%s setupQUAD ok\n",__func__);
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
        _INFOLIST("%s Bands[%d]=%d quad=%d\n",__func__,Band_slot,s,q);
     #endif //DEBUG   

   #endif //QUAD      
   

/*------
 * Check if calibration is needed
 */

   if (detectKey(DOWN,LOW,WAIT)==LOW) { 
      #ifdef DEBUG
        _INFOLIST("%s Calibration mode detected\n",__func__);
      #endif //DEBUG
      #ifdef AUTOCAL       //Automatic calibration
          setWord(&QSW,QCAL,true);       
          setWord(&QSW,QWAIT,true);
          while (true) {
            #ifdef WDT
               wdt_reset();
            #endif //WDT   
          }
      #else   //Manual calibration
         #ifdef ADX
            Calibration();
         #endif //ADX   
      #endif //AUTOCAL   
   }
  
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
     _INFOLIST("%s Counting algorithm TIMER1 set Ok\n",__func__);
  #endif //DEBUG   
  
#endif //ADX

#ifdef PDX
  /*------------------------------------*
   * trigger counting algorithm         *
   *------------------------------------*/
  rp2040.idleOtherCore();
  #ifdef DEBUG
      _INFOLIST("%s Core1 stopped ok\n",__func__);
  #endif //DEBUG   

  setWord(&QSW,QFSK,true);
  setWord(&QSW,QWAIT,true);
  #ifdef DEBUG
      _INFOLIST("%s FSK detection algorithm started QFSK=%s QWAIT=%s ok\n",__func__,BOOL2CHAR(getWord(QSW,QFSK)),BOOL2CHAR(getWord(QSW,QWAIT)));
  #endif //DEBUG   
  delay(500);
  

#endif //PDX

  switch_RXTX(LOW);
  #ifdef DEBUG
      _INFOLIST("%s switch_RXTX Low ok\n",__func__);
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
     #ifdef DEBUG
        _INFOLIST("%s watchdog configuration completed\n",__func__);
     #endif //DEBUG   

  #endif //WDT



#ifdef TERMINAL
/*------------------
 * if UP switch pressed at bootup then enter Terminal mode
 */
   if (detectKey(UP,LOW,WAIT)==LOW) { 
      execTerminal();      
   }  
#endif //TERMINAL   


/*------------
 * re-start the core1 where the FSK counting is performed
 */
  #ifdef PDX
     rp2040.restartCore1();
     delay(1); 
     #ifdef DEBUG
        _INFOLIST("%s Core1 resumed ok\n",__func__);
     #endif //DEBUG   
  #endif //PDX

  #ifdef DEBUG
     _INFOLIST("%s watchdog configuration completed\n",__func__);
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
 *  Manage the user interface (UP/DOWN/TX switches & combinations)                 *
 *  Change frequency, mode, band                                                   *
 *---------------------------------------------------------------------------------*/  
    checkMode();
/*---------------------------------------------------------------------------------*
 *  Save EEPROM if a change has been flagged anywhere in the logic                 *
 *---------------------------------------------------------------------------------*/  
    #ifdef EE
//*--- if EEPROM enabled check if timeout to write has been elapsed
    checkEEPROM();
    #endif //EEPROM

/*---------------------------------------------------------------------------------*
 *  ATU pulse width control, reset signal after the elapsed time elapsed happens   *
 *---------------------------------------------------------------------------------*/  
   #ifdef ATUCTL
   if ((millis()-tATU)>atu_delay && getWord(TSW,ATUCLK)==true) {
       setWord(&TSW,ATUCLK,false);
       setGPIO(atu,LOW);
    }
    #endif //ATUCTL       

/*---------------------------------------------------------------------------------*
 *  Sample for CAT commands if enabled                                             *
 *---------------------------------------------------------------------------------*/  
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

//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                                                                                *
//*                      ADX Counting Algorithm                                    *
//*                                                                                *
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
/*---------------------------------------------------------------------------------*
 * Timer1 (16 bits) with no pre-scaler (16 MHz) is checked to detect zero crossings*
 * if there is no overflow the frequency is calculated                             *
 * if activity is detected the TX is turned on                                     *
 * TX mode remains till no further activity is detected (operate like a VOX command*
 *---------------------------------------------------------------------------------*/
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

             if (getWord(SSW,VOX) == false){
                 switch_RXTX(HIGH);                 
             }

             si5351.set_freq(((freq + codefreq) * 100ULL), SI5351_CLK0); 
             setWord(&SSW,VOX,true);

             #ifdef WDT
                wdt_reset();
             #endif //WDT
       }
       
    } else {
       n--;
    }

   /*----------------------*
    * Sample CAT commands  *
    *----------------------*/   
    #ifdef CAT 
//*--- if CAT enabled check for serial events (again)
       serialEvent();
    #endif
    
    #ifdef WDT
       wdt_reset();
    #endif //WDT
 }
#endif //ADX

#ifdef PDX
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                                                                                *
//*                      PDX Counting Algorithm                                    *
//*                                                                                *
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
/*---------------------------------------------------------------------------------*
 * setup1 () is running on a different thread at core1 and sampling the frequency  *
 * using either a pwm counting (FSK_PEG) or a pseudo zero crossing (FSK_ZCD) method*
 * Whenever the frequency falls within the [FSKMIN,FSKMAX] limits it's FIFOed here *
 * Additional heuristic of validation are also applied to manage counting common   *
 * counting errors.                                                                *
 * FSK_PEG                                                                         *
 * The counting algorithm has a common error of +/- 1 count because of the moment  *
 * the sampling starts (which might include or exclude one edge), because of the   *
 * window measurement applied this is translated into a +/- FSK_MULT (Hz) error    *
 * This value needs to be much larger than the maximum bandwidth of the signal to  *
 * be decoded. i.e. With FSK_WINDOW at 10 mSec FSK_MULT is 100 thus the count      * 
 * error can be up to +/- 100 Hz. As the FT8 signal occupies up to 50 Hz then the  *
 * deviation is produced by a counting common error and not of a PSK tone change   *
 * and thus ignored. Other counting errors can produce an actual shift of the      *
 * transmitting frequency and thus a decoding issue on the other side. Actual      *
 * measurement seems to point to error<0.2%                                        *
 * FSK_ZCD                                                                         *
 * The counting algorithm has rounding errors in the range of <500 uSec because the*
 * error in the triggering level and the residual +/- 1 uSec counting error        *
 * the frequency is filtered by the bandwidth level and also a rounding error of   *
 * 1 Hz, thus if the sampled frequency is off by +/- 1 Hz the difference is less   *
 * than the change on the PSK tone and thus ignored                                *
 *---------------------------------------------------------------------------------*/
uint16_t n = VOX_MAXTRY;
uint32_t qBad=0;    
uint32_t qTot=0;
boolean  f=true;

    setWord(&SSW,VOX,false);
    while ( n > 0 ){                                 //Iterate up to 10 times looking for signal to transmit

/*-----------------------------*
 * if enabled manage watchdog  *
 *-----------------------------*/
    #ifdef WDT
       wdt_reset();

       if (getWord(TSW,TX_WDT)==HIGH) {
           break;
       }  //If watchdog has been triggered so no TX is allowed till a wdt_max timeout period has elapsed   
       
       if ((millis() > (wdt_tout+uint32_t(WDT_MAX))) && getWord(SSW,TXON) == HIGH) {
          switch_RXTX(LOW);
          setWord(&TSW,TX_WDT,HIGH);
          wdt_tout=millis();
          #ifdef DEBUG 
             _INFOLIST("%s TX watchdog condition triggered\n",__func__);
          #endif //DEBUG
          break;
       }
    #endif //WDT
/*-----------------------------------------------------*
 * frequency measurements are pushed from core1 when   *
 * a sample is available. If no signal is available    *
 * no sample is provided. Thus it's wait for a number  *
 * of cycles till extingish the TX mode and fallback   *
 * into RX mode.                                       *
 *-----------------------------------------------------*/
    if (rp2040.fifo.available() != 0) {             
        codefreq=rp2040.fifo.pop();
        /*------------------------------------------------------*
         * Filter out frequencies outside the allowed bandwidth *
         *------------------------------------------------------*/
        if (codefreq >= uint32_t(FSKMIN) && codefreq <= uint32_t(FSKMAX)) {
           n=VOX_MAXTRY;
           qTot++;

           /*----------------------------------------------------*
            * if VOX is off then pass into TX mode               *
            * Frequency IS NOT changed on the first sample       *
            *----------------------------------------------------*/
   
           if (getWord(SSW,VOX)==false) {                            
              #ifdef DEBUG
                  _INFOLIST("%s VOX activated n=%d f=%ld\n",__func__,n,codefreq);
              #endif //DEBUG                                  
              switch_RXTX(HIGH); 
              prevfreq=codefreq;
              setWord(&SSW,VOX,true);
              continue;
           }
           /*-----------------------------------------------------*
            * If this is the first sample AFTER the one that set  *
            * the VOX on then switch the frequency to it          *
            *-----------------------------------------------------*/
           if (f==true) {           
              si5351.set_freq(((freq + codefreq) * 100ULL), SI5351_CLK0); 
              prevfreq=codefreq;            
             _INFOLIST("%s Freq sample first f=%ld prev=%ld\n",__func__,codefreq,prevfreq);
             f=false;
             continue;
           }
           /*------------------------------------------------------*
            * Strategy to correct common errors depending on the   *
            * method used for counting                             *
            *------------------------------------------------------*/

           /*----
            * Strategy for ZCD
            *----*/
           #ifdef FSK_ZCD
             int d=codefreq-prevfreq;
             if (abs(d) > FSK_ERROR) {
                si5351.set_freq(((freq + codefreq) * 100ULL), SI5351_CLK0); 
                if (codefreq != prevfreq) {
                   #ifdef DEBUG
                     _INFOLIST("%s Freq sample changed f=%ld prev=%ld\n",__func__,codefreq,prevfreq);
                     qBad++;
                   #endif //DEBUG                                           
                   prevfreq=codefreq;
                }                   
              }
           #endif //FSK_ZCD

           /*----
            * Strategy for PEG
            *----*/
           #ifdef FSK_PEG
             int d=codefreq-prevfreq;
                if ((abs(d)<=FSK_MULT-1)) {
                si5351.set_freq(((freq + codefreq) * 100ULL), SI5351_CLK0); 
                if (codefreq != prevfreq) {
                   #ifdef DEBUG
                     _INFOLIST("%s Freq sample changed f=%ld prev=%ld\n",__func__,codefreq,prevfreq);
                     qBad++;
                   #endif //DEBUG                                           
                   prevfreq=codefreq;
                }                   
              }
           #endif //FSK_PEG
        }            
        /*----------------
         * Watchdog reset
         *---------------*/ 
         #ifdef WDT
           wdt_reset();
         #endif //WDT
    } else {
         /*--------------------
          * Waiting for signal
          *--------------------*/
         uint32_t tcnt = time_us_32() + uint32_t(FSK_IDLE);
         while (tcnt > time_us_32());
         n--;
    }

    /*----------------------*
     * Sample CAT commands  *
     *----------------------*/
    #ifdef CAT 
       serialEvent();
    #endif
    
    /*----------------------*
     * Sample watchdog reset*
     *----------------------*/   
    #ifdef WDT
       wdt_reset();
    #endif //WDT
 }
 
/*---------------------------------------------------------------------------------*
 * when out of the loop no further TX activity is performed, therefore the TX is   *
 * turned off and the board is set into RX mode                                    *
 *---------------------------------------------------------------------------------*/

 /*------------------------------*
  * This is a development probe  *
  * to measure the counting      *
  * error obtained into the      *
  * frequency checking           *
  *------------------------------*/
 #ifdef DEBUG
 if (qTot != 0) {
    float r=100.0*(float(qBad*1.0)/float(qTot*1.0));
    #ifdef DEBUG
       _INFOLIST("%s <eof> qBad=%ld qTot=%ld error=%.6f\n",__func__,qBad,qTot,r);   
    #endif //DEBUG
    qBad=0;
    qTot=0;
 }
#endif //DEBUG    

#endif //PDX    

//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                               RX Cycle                                               *
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
/*---------------------------------------------------------------------------------------*
 * TX cycle ends, fallback to RX mode                                                    *
 *---------------------------------------------------------------------------------------*/

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
    #ifdef DEBUG
       _INFOLIST("%s TX watchdog condition cleared\n",__func__);
    #endif //DEBUG
    #ifdef PDX
        /*-----
         * Clear FIFO
         *-----*/
        while (rp2040.fifo.available() != 0) {             
           uint32_t dummy=rp2040.fifo.pop();
        }
    #endif //PDX
 }
#endif //WDT

/*----------------------*
 * Sample CAT commands  *
 *----------------------*/   
#ifdef CAT
 serialEvent();
#endif //CAT 

/*------------------------------------------------------------*
 * At this point it must be in RX mode so perform the switch  *
 *------------------------------------------------------------*/   
 if (getWord(SSW,CATTX)!=true) {
    switch_RXTX(LOW);
    setWord(&SSW,VOX,false);
    setWord(&SSW,TXON,false);
 }   

/*----------------------*
 * Reset watchdog       *
 *----------------------*/   
 #ifdef WDT
    wdt_reset();
 #endif //WDT     

}
//****************************[ END OF MAIN LOOP FUNCTION ]*******************************
//********************************[ END OF FIRMWARE ]*************************************
