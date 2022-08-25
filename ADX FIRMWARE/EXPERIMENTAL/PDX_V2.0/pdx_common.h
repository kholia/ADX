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



//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                            VERSION HEADER                                                   *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
#define VERSION        "2.0"
#define BUILD          3

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                       External libraries used                                               *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
#include <stdint.h>
#include <si5351.h>

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
#define WDT               1      //Hardware and TX watchdog enabled
#define EE                1      //Save in Flash emulation of EEPROM the configuration
#define CAT               1      //Enable CAT protocol over serial port
#define FT817             1      //Yaesu FT817 CAT protocol
#define TERMINAL          1      //Serial configuration terminal used
#define QUAD              1      //Support for QUAD board
#define ATUCTL            1      //Brief 200 mSec pulse to reset ATU on each band change

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
#define ATU_DELAY     200       //How long the ATU control line (D5) is held HIGH on band changes, in mSecs


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
   UART Pin
*/
#define UART_TX        12
#define UART_RX        13

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

//#if (defined(CAT) && defined(DEBUG))  //Rule for conflicting usage of the serial port
//#undef  DEBUG
//#endif // CAT && DEBUG

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

/*----------------------------------------
 * Callback structure
 */
// --- IPC structures

struct msg {
       char timestamp[16];    // time stamp
       byte slot;             //
       bool CQ;
        int snr;              // Signal SNR
      float DT;               // Time Shift
        int offset;           // Frequency offset
       char t1[16];           // first token
       char t2[16];           // second token
       char t3[16];           // third token
};

struct header {
      char  timestamp[16];
      byte  slot;
      bool  newheader;
      bool  active;
};

struct qso {
      byte  FSM;
      int   nmsg;
      char  hiscall[16];
      char  hisgrid[16];
      char  hissnr[16];
      int   hisslot;
      int   offset;
      char  mycall[16];
      char  mygrid[16];
      char  mysnr[16];
      int   myslot;
      int   cnt;
      char  lastsent[256];
};
struct Queue {
      int   slot;
      char  message[1024];
};

typedef void (*CALLBACK)();
typedef void (*QSOCALL)(header* h,msg* m,qso* q);

extern void    setupCallback(CALLBACK s,QSOCALL q);
extern void    doupCall();

extern QSOCALL FSMHandler;
extern CALLBACK upCall;

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*               DEBUG SUPPORT MACRO DEFINITIONS                                               *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*

/*****************************************************************
   Trace and debugging macros (only enabled if DEBUG is set
 *****************************************************************/
//#define DEBUG  1

#ifdef DEBUG        //Remove comment on the following #define to enable the type of debug macro
//#define INFO   1   //Enable _INFO and _INFOLIST statements
//#define EXCP   1   //Enable _EXCP and _EXCPLIST statements
//#define TRACE  1   //Enable _TRACE and _TRACELIST statements
//#define CORE   1   //Enable _CORE2 and _CORE2LIST statements (debugging of core 2 code)
#endif //DEBUG

/*------
 * can not simultaneously use the serial port from both core, either one or the other will
 * have that possibility. If CORE2 trace is made then no other trace is made
 * otherwise a race condition, lockouts and buffer corruption might happen.
 */
#if (defined(DEBUG) && (defined(CORE2)))
#undef EXCP
#undef INFO
#undef TRACE
#endif //DEBUG && CORE2

/*-------------------------------------------------------------------------*
   Define Info,Exception and Trace macros (replaced by NOP if not enabled)
  -------------------------------------------------------------------------*/

/*---
 * Define which serial port will be used for debugging, Serial is the USB serial
 * whilst Serial1 is the UART based serial at pins GPIO16/17 (much more convenient
 * for debugging purposes)
 */
#ifdef DEBUG
#define DEBUG_UART 1
#ifdef DEBUG_UART
#define _SERIAL Serial1
#else 
#define _SERIAL Serial
#endif //DEBUG_UART
#endif //DEBUG

#ifdef DEBUG
#define _serial1(...)   sprintf(hi,__VA_ARGS__);_SERIAL.write(hi);_SERIAL.flush();
#else
#define _serial1(...)   _NOP
#endif

#ifdef TRACE
#define _TRACE           sprintf(hi,"%s: Ok\n",__func__); _SERIAL.print(strcat("@",hi));_SERIAL.flush();
#define _TRACELIST(...)  strcpy(hi,"@");sprintf(hi+1,__VA_ARGS__);_SERIAL.print(hi);_SERIAL.flush();
#else
#define _TRACE _NOP
#define _TRACELIST(...)  _TRACE
#endif

#ifdef CORE2
#define _CORE2           sprintf(hi,"%s: Ok\n",__func__); _SERIAL.print(strcat("@",hi));_SERIAL.flush();
#define _CORE2LIST(...)  strcpy(hi,"@");sprintf(hi+1,__VA_ARGS__);_SERIAL.print(hi);_SERIAL.flush();
#else
#define _CORE2 _NOP
#define _CORE2LIST(...)  _CORE2
#endif //CORE2

#ifdef INFO
#define _INFO           sprintf(hi,"@%s: Ok\n",__func__); _SERIAL.print(hi);_SERIAL.flush();
#define _INFOLIST(...)  strcpy(hi,"@");sprintf(hi+1,__VA_ARGS__);_SERIAL.print(hi);_SERIAL.flush();
#else
#define _INFO _NOP
#define _INFOLIST(...)  _INFO
#endif

#ifdef EXCP
#define _EXCP           sprintf(hi,"%s: Ok\n",__func__); _SERIAL.print(strcat("@",hi));_SERIAL.flush();
#define _EXCPLIST(...)  strcpy(hi,"@");sprintf(hi+1,__VA_ARGS__);_SERIAL.print(strcat("@",hi));_SERIAL.flush();
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
extern uint8_t        TSW;
extern uint8_t        QSW;

extern unsigned long  freq;
extern int            setSlot(uint32_t f);
extern uint16_t       Band_slot;
extern uint16_t       mode;
extern const uint16_t Bands[BANDS];


extern uint16_t       bounce_time;
extern uint16_t       short_time;
extern uint16_t       vox_maxtry;
extern int            cnt_max;
extern uint16_t       max_blink;
extern int32_t        cal_factor;
extern unsigned long  Cal_freq; // Calibration Frequency: 1 Mhz = 1000000 Hz

extern double         fsequences[NFS]; // Ring buffer for communication across cores
extern int            nfsi;
extern double         pfo; // Previous output frequency





/*-----------
   Function references

*/
extern const uint8_t  LED[4];
extern void           rstLED(uint8_t LEDpin, bool clrLED);
extern void           resetLED();
extern void           setLED(uint8_t LEDpin, bool clrLED);
extern void           blinkLED(uint8_t LEDpin);
extern void           calibrateLED();

extern void           Mode_assign();
extern void           switch_RXTX(bool t);

extern bool           getWord (uint8_t SysWord, uint8_t v);
extern void           setWord(uint8_t* SysWord, uint8_t v, bool val);
extern void           delay_uSec(uint16_t d);

extern void           serialEvent();


extern int            getBand(uint32_t f);
extern int            findSlot(uint16_t i);
extern uint8_t        band2Slot(uint16_t k);
extern int            getMode(int i, uint32_t f);
extern uint16_t       changeBand(uint16_t c);
extern void           Band_assign(bool l);
extern void           setStdFreq(int i);
extern void           resetBand(int bs);

extern void           execTerminal();
extern void           printMessage(char * token);
extern int            updateFreq(uint32_t fx);

#ifdef ATUCTL
extern void           initATU();
extern void           flipATU();
extern void           checkATU();
extern uint16_t       atu;
extern uint16_t       atu_delay;
extern uint32_t       tATU;
#endif //ATUCTL

#ifdef EE
extern void           updateEEPROM();
extern void           resetEEPROM();
extern void           checkEEPROM();
extern void           flagEEPROM();
extern void           initEEPROM();
extern uint16_t       eeprom_tout;
extern uint32_t       tout;
#endif //EE

#ifdef QUAD
extern int            band2QUAD(uint16_t b);
extern void           setQUAD(int LPFslot);
#endif //QUAD

#ifdef WDT
extern uint32_t      wdt_tout;
#endif //WDT


extern const unsigned long slot[MAXBAND][MAXMODE];
extern unsigned long f[MAXMODE];

extern Si5351         si5351;

extern uint32_t       ffsk;
extern int            pwm_slice;
extern uint32_t       f_hi;
extern uint32_t       fclk;
extern int32_t        error;
extern uint32_t       codefreq;
extern uint32_t       prevfreq;


/*------------------------------*
   Epoch values
  ------------------------------*/
extern uint32_t t1[2];
extern uint32_t t2[2];
extern uint32_t v1[2];
extern uint32_t v2[2];

#ifdef ADCZ
/*-------------------------------*
   Sample data
*/
extern uint16_t adc_v1;
extern uint16_t adc_v2;
extern uint32_t adc_t1;
extern uint32_t adc_t2;

extern bool     adc_high;
extern bool     adc_low;

/*-------------------------------*
   Computed frequency limits
*/
extern double   ffmin;
extern double   ffmax;
extern uint16_t adc_min;
extern uint16_t adc_max;
extern uint16_t adc_zero;
extern uint16_t adc_uh;
extern uint16_t adc_ul;
/*--------------------------------*
   FSM state variable
  --------------------------------*/
extern uint8_t  QSTATE;
#endif //ADCZ
#endif //
