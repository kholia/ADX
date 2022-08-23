//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=**=*=*
//*                           PDX - PICO based DIGITAL MODES 4 BAND HF TRANSCEIVER                           *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=**=*=*
// Firmware Version 2.0
//
// Barb (Barbaros ASUROGLU) - WB2CBA - 2022
// Dhiru (Dhiru Kholia)     - VU3CER - 2022
// Pedro (Pedro Colla)      - LU7DZ  - 2022
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=**=*=*
// Common configuration resources and definitions
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=**=*=*
#ifndef PDX_common
#define PDX_common

#include <stdint.h>
#include "hardware/watchdog.h"
#include "Wire.h"
#include <EEPROM.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"
#include <stdio.h>
#include "hardware/pwm.h"
#include "pico/multicore.h"
#include "hardware/adc.h"
#include "hardware/uart.h"


//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                            MACRO DEFINES                                                    *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
#define BOOL2CHAR(x)  (x==true ? "True" : "False")
#undef  _NOP
#define _NOP          (byte)0

#define resetFunc() while(true) {}
#define getGPIO(x) gpio_get(x)
#define setGPIO(x,y) gpio_put(x,y)
#define PICODISPLAY 1
#define wdt_reset() watchdog_update()

#define FT817      1
//#define TS480      1

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                               (P)ico (D)igital (X)ceiver                                    *
//*                            FEATURE CONFIGURATION PROPERTIES                                 *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//#define WDT             1      //Hardware and TX watchdog enabled
//#define EE              1      //Save in Flash emulation of EEPROM the configuration
#define CAT             1      //Enable CAT protocol over serial port
//#define QUAD            1      //Support for QUAD board
//#define ATUCTL          1      //Brief 200 mSec pulse to reset ATU on each band change

//#define CW              1      //CW support
//#define ONEBAND         1      //Define a single band

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                      GENERAL PURPOSE GLOBAL DEFINITIONS                                     *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
#define BANDS          4            //Max number of bands allowed
#define BOUNCE_TIME    200           //mSec minimum to debounce
#define SHORT_TIME     10*BOUNCE_TIME //mSec minimum to consider long push
#define SI5351_REF     25000000UL   //change this to the frequency of the crystal on your si5351’s PCB, usually 25 or 27 MHz
#define CPU_CLOCK      16000000UL   //Processor clock
#define VOX_MAXTRY     15           //Max number of attempts to detect an audio incoming signal
#define CNT_MAX        65000        //Max count of timer1
#define FRQ_MAX        30000        //Max divisor for frequency allowed
#define BDLY           200          //Delay when blinking LED
#define DELAY_WAIT     BDLY*2       //Double Delay
#define DELAY_CAL      DELAY_WAIT/10
#define MAXMODE        5            //Max number of digital modes
#define MAX_BLINK      4            //Max number of blinks
#define MAXBAND       9            //Max number of bands defined (actually uses BANDS out of MAXBAND)
#define XT_CAL_F      33000         //Si5351 Calibration constant
#define CAL_STEP      500           //Calibration factor step up/down while in calibration (sweet spot experimentally found by Barb)
#define REPEAT_KEY    30            //Key repetition period while in calibration
#define WAIT          true          //Debouncing constant
#define NOWAIT        false         //Debouncing constant
#define SERIAL_TOUT   5000
#define SERIAL_WAIT   2
#define CAT_RECEIVE_TIMEOUT 500
#define numChars 256

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                      PIN ASSIGNMENTS                                                        *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
/*----
   Output control lines
*/
#define RX              2      //RX Switch
#ifdef ATUCTL
#define ATU            15     //ATU Device control line (flipped HIGH during 200 mSecs at a band change)
#endif //ATUCTL

/*---
   LED
*/
#define WSPR            7      //WSPR LED
#define JS8             6      //JS8 LED
#define FT4             5      //FT4 LED
#define FT8             4      //FT8 LED
#define TX              3      //TX LED
/*---
   Switches
*/
#define UP             10      //UP Switch (this must be set to GPIO19 when running on a PDX board)
#define DOWN           11      //DOWN Switch (this must be set to GPIO20 when running on a PDX board)
#define TXSW            8      //TX Switch
/*---
    I2C
*/
#define PDX_I2C_SDA    16      //I2C SDA
#define PDX_I2C_SCL    17      //I2C SCL
/*---
    Input lines
*/
#define FSK            27      //Frequency counter algorithm
#define CAL             9      //Automatic calibration entry

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                      GLOBAL STATE VARIABLE DEFINITIONS                                      *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
/*----------------------------------------------------------------*
    Global State Variables (Binary)
  ----------------------------------------------------------------*/
#define TXON        0B00000001    //State of the TX
#define VOX         0B00000010    //Audio input detected
#define UPPUSH      0B00000100    //UP button pressed
#define DNPUSH      0B00001000    //DOWN button pressed
#define TXPUSH      0B00010000    //TXSW button pressed
#define CATTX       0B00100000  // TX turned on via CAT (disable VOX)
#define SAVEEE      0B01000000    //Mark of EEPROM updated
#define CWMODE      0B10000000    //CW Mode active
/*----------------------------------------------------------------*
   Operating switch
   ---------------------------------------------------------------*/
#define PUSHSTATE   0B00000001
#define SHORTPUSH   0B00000010    //simple push flag
#define LONGPUSH    0B00000100    //long push flag
#define INPROGRESS  0B00001000    //in progress mark
#define ATUCLK      0B00010000    //control the width of the ATU pulse
#define TX_WDT      0B00100000    //TX Watchdog has been activated
#define AVOX        0B01000000    //ANTI-VOX has been activated
#define UNUSED      0B10000000    //Counter mode semaphore

/*----------------------------------------------------------------*
   IPC Management
   ---------------------------------------------------------------*/
#define QWAIT       0B00000001    //Semaphore Wait
#define QCAL        0B00000010    //Calibration (using 2 cores)
#define QFSK        0B00000100    //FSK detection

/*----------------------------------------------------------------*
   Miscellaneour definitions
   ---------------------------------------------------------------*/
#define BAUD            38400

#define INT0                0
#define INT1                1
#define INT2                2
#define RTIME            2000
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*               DEFINITIONS SPECIFIC TO THE RP2040 ARCHITECTURE                               *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
#define  CAL_COMMIT      12
#define  CAL_ERROR       1

//#define  FSK_ADCZ        1
#if !defined(FSK_ADCZ)
#define FSK_ZCD      1
#endif

#define FSKMIN             300    //Minimum FSK frequency computed
#define FSKMAX            3000    //Maximum FSK frequency computed
#ifdef FSK_ZCD

#define FSK_USEC                  1000000
#define FSK_SAMPLE                   1000
#define FSK_ERROR                       4
#define FSK_RA                         20
#define FSK_IDLE      5*FSK_SAMPLE*FSK_RA

#endif //FSK_ZCD


#ifdef FSK_ADCZ
/* Extract analog values from the selected ADC pin
   The signal might have any level of DC and might have some noise
   so a strict finite state machine (FSM) is used to ensure that only
   valid states are included in the frequency computation
   Still, small measurement errors might lead to the wrong computation of
   the frequency.
*/

/*------------------------------*
   ADC port values
  ------------------------------*/
#define ADC_NUM 1
#define ADC_PIN (26 + ADC_NUM)
#define ADC_VREF 3.3
#define ADC_RANGE (1 << 12)
#define ADC_CONVERT (ADC_VREF / (ADC_RANGE - 1))

/*------------------------------*
   ADC Limits and operation
  ------------------------------*/
#define  ADCMAX      4096
#define  ADCMIN         0
#define  ADCZERO     (ADCMAX+ADCMIN)/2
#define  ADCSAMPLE      1
#define  FSK_IDLE       1000

#endif //FSK_ADCZ
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

#if (defined(CAT) && (!defined(FT817) && !defined(TS480)))
#define FT817      1
#endif // CAT && FT817 forced if no CAT protocol indicated

#if (defined(CAT) && defined(FT817))
#undef TS480
#endif // CAT && TS480

//*--- if both supported CAT protocols are simultaneously selected then keep one
#define NFS 32
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*               DEBUG SUPPORT MACRO DEFINITIONS                                               *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*

/*****************************************************************
   Trace and debugging macros (only enabled if DEBUG is set
 *****************************************************************/

// #define DEBUG  1
#ifdef DEBUG        //Remove comment on the following #define to enable the type of debug macro
#define INFO  1   //Enable _INFO and _INFOLIST statements
//#define EXCP  1   //Enable _EXCP and _EXCPLIST statements
//#define TRACE 1   //Enable _TRACE and _TRACELIST statements
#endif //DEBUG
/*-------------------------------------------------------------------------*
   Define Info,Exception and Trace macros (replaced by NOP if not enabled)
  -------------------------------------------------------------------------*/
#ifdef DEBUG
#define _DEBUG           sprintf(hi,"@%s: Ok\n",__func__); Serial.print(hi);
#define _DEBUGLIST(...)  strcpy(hi,"@");sprintf(hi+1,__VA_ARGS__);Serial.print(hi);
#define print2(x,y) (Serial.print(x), Serial.println(y))

#else
#define _DEBUG _NOP
#define _DEBUGLIST(...)  _DEBUG
#define print2(x,y) _DEBUG
#endif

#ifdef TRACE
#define _TRACE           sprintf(hi,"%s: Ok\n",__func__); Serial.print(strcat("@",hi));
#define _TRACELIST(...)  strcpy(hi,"@");sprintf(hi+1,__VA_ARGS__);Serial.print(hi);
#else
#define _TRACE _NOP
#define _TRACELIST(...)  _TRACE
#endif

#ifdef INFO
#define _INFO           sprintf(hi,"@%s: Ok\n",__func__); Serial.print(hi);delay(40);Serial.flush();
#define _INFOLIST(...)  strcpy(hi,"@");sprintf(hi+1,__VA_ARGS__);Serial.print(hi);delay(40);Serial.flush();
#else
#define _INFO _NOP
#define _INFOLIST(...)  _INFO
#endif

#ifdef EXCP
#define _EXCP           sprintf(hi,"%s: Ok\n",__func__); Serial.print(strcat("@",hi));
#define _EXCPLIST(...)  strcpy(hi,"@");sprintf(hi+1,__VA_ARGS__);Serial.print(strcat("@",hi));
#else
#define _EXCP           _NOP
#define _EXCPLIST(...)  _EXCP
#endif
#ifdef CW
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*       CW FEATURE IF PTT IS ACTIVATED BY BUTTON/CAT THE TX FREQUENCY WILL BE SHIFT UP        *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
#define CWSHIFT       600
uint16_t cwshift = CWSHIFT;
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
#define EEPROM_EEPROM_TOUT 10000   //Timeout to commit to EEPROM any change
#define EEPROM_AVOXTIME    170
#define EEPROM_END         200
#endif //TERMINAL
//#define EEPROM_CLR     1   //Initialize EEPROM (only to be used to initialize contents)
#define EEPROM_SAVED  100     //Signature of EEPROM being updated at least once
#define EEPROM_TOUT  2000     //Timeout in mSecs to wait till commit to EEPROM any change
#endif //EEPROM
#ifdef WDT
#define       WDT_MAX     130000
#endif //WDT
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*                Definitions shared by all sub-systems
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
/*------------
   Memory references
*/
extern char           hi[80];
extern uint8_t        SSW;
extern unsigned long  freq;
extern int            setSlot(uint32_t f);
extern uint16_t       Band_slot;
extern uint16_t       mode;
extern const uint16_t Bands[BANDS];

/*-----------
   Function references

*/
//@@@extern void           Freq_assign();
extern int            getBand(uint32_t f);
extern void           Mode_assign();
extern int            findSlot(uint16_t i);
extern uint8_t        band2Slot(uint16_t k);
extern int            getMode(int i, uint32_t f);
extern bool           getWord (uint8_t SysWord, uint8_t v);
extern void           setWord(uint8_t* SysWord, uint8_t v, bool val);
extern uint16_t       changeBand(uint16_t c);
extern void           Band_assign(bool l);
extern void           flipATU();
extern void           delay_uSec(uint16_t d);
extern void           serialEvent();
extern void           switch_RXTX(bool t);
extern void           setStdFreq(int i);
extern void           resetBand(int bs);

#ifdef CAT
extern int            updateFreq(uint32_t fx);
#endif //CAT

#ifdef QUAD
extern int            band2QUAD(uint16_t b);
extern void           setQUAD(int LPFslot);
#endif //QUAD

#ifdef WDT
extern uint32_t      wdt_tout;
#endif //WDT

#ifdef EE
extern uint32_t      tout;
#endif //EE

extern const unsigned long slot[MAXBAND][MAXMODE];
extern unsigned long f[MAXMODE];
#endif
