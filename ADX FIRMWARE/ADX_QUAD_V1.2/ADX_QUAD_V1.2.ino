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
// AVR "wdt.h" Watchdog Library
//*************************************[ LICENCE and CREDITS ]*********************************************
//  FSK TX Signal Generation code by: Burkhard Kainka(DK7JD) - http://elektronik-labor.de/HF/SDRtxFSK2.html
//  SI5351 Library by Jason Mildrum (NT7S) - https://github.com/etherkit/Si5351Arduino
//*-----------------------------------------------------------------------------------------------------------------*
//* Modified by Dr. P.E.Colla (LU7DZ)                                                                               
//*     X re-style of the code to facilitate customization for multiple boards
//*     X Add all frequency definitions for HF bands
//*     X Optimize EEPROM read/write cycles
//*     X changes to compatibilize with Pixino board (http://www.github.com/lu7did/Pixino
//*     / add CW support (includes keyer support)
//*     X add CAT support (TS-440)
//*     X add timeout & watchdog support
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
/*-----------------------------------------------------------*
 * System Includes
 *-----------------------------------------------------------*/
#include <stdint.h>
#include <avr/wdt.h> 
#include <si5351.h>
#include "Wire.h"
#include <EEPROM.h>
//********************************[ DEFINES ]***************************************************
#define VERSION     "1.2a"
#define BOOL2CHAR(x)  (x==true ? "True" : "False")
/*****************************************************************
 * CONSISTENCY RULES                                             *
 *****************************************************************/
//#define ECHO        1
#define WDT         1     //Hardware watchdog enabled
#define DEBUG       1     //DEBUG is nullified when CAT is enabled to avoid conflicts
#define EE          1     //User EEPROM for persistence
//#define CAT         1     //Emulates a TS-440 transceiver CAT protocol
#define CW          1

#if (defined(DEBUG) || defined(CAT))
    char hi[120];
    uint32_t tx=0;
    #define _SERIAL 1
    #define BAUD_DEBUG 115200
#endif //DEBUG or CAT


#if (defined(CAT) && defined(DEBUG))  //Rule for conflicting usage of the serial port
    #undef  DEBUG
#endif // CAT && DEBUG

#ifdef CAT
   #define CATCMD_SIZE          32
   #define BAUD_CAT 9600
   char CATcmd[CATCMD_SIZE];
   volatile uint8_t cat_active = 0;
   volatile uint32_t rxend_event = 0;
#endif

/*----------------------------*
 * Pin Assignment             *
 *----------------------------*/
#define AIN0           6           //(PD6)
#define AIN1           7           //(PD7)
#define TX            13           //(PB5) TX LED
#define UP             2           //UP Switch
#define DOWN           3           //DOWN Switch
#define TXSW           4           //TX Switch
#define RX             8           //RX Switch
#define WSPR           9           //WSPR LED 
#define JS8           10           //JS8 LED
#define FT4           11           //FT4 LED
#define FT8           12           //FT8 LED

/*-------------------------------------------*
 *  Global State Variables (Binary)          *
 *-------------------------------------------*/
#define TXON   0B00000001    //State of the TX
#define VOX    0B00000010    //Audio input detected
#define UPPUSH 0B00000100    //UP button (analog) pressed
#define DNPUSH 0B00001000    //DOWN button (analog) pressed
#define TXPUSH 0B00010000    //TXSW button (analog) pressed
#define PL     0B00100000    //Long pulse detected on any analog button
#define SAVEEE 0B01000000    //Mark of EEPROM updated
#define CWMODE 0B10000000    //CW Active

/*------------------------------------*
 * General purpose global define      *
 * -----------------------------------*/
#define SI5351_REF  25000000UL  //change this to the frequency of the crystal on your si5351’s PCB, usually 25 or 27 MHz
#define CPU_CLOCK   16000000UL  //Processor clock
#define VOX_MAXTRY  10          //Max number of attempts to detect an audio incoming signal
#define CNT_MAX     65000       //Max count of timer1
#define FRQ_MAX     30000       //Max divisor for frequency allowed
#define BDLY        200
#define DELAY_WAIT  BDLY*2
#define DELAY_CAL   DELAY_WAIT/10

#ifdef EE
   #define EEPROM_CAL  10
   #define EEPROM_TEMP 30
   #define EEPROM_MODE 40
   #define EEPROM_BAND 50

   uint32_t tout=0;

   //#define EEPROM_CLR    1   //Initialize EEPROM (only to be used to initialize contents)
   #define EEPROM_SAVE 100     //Signature of EEPROM being updated at least once
   #define EEPROM_TOUT 500     //Timeout in mSecs to wait till commit to EEPROM any change
#endif //EEPROM

//*******************************[ VARIABLE DECLARATIONS ]*************************************
#define  MAXMODE          4
#define  MAXBLINK         4
#define  MAXBAND         10
uint8_t  SSW=0;               //System SSW variable (to be used with getSSW/setSSW)
uint16_t mode=0;              //Default to mode=0 (FT8)
uint16_t Band_slot=0;         //Default to Bands[0]=40
uint16_t cal_factor=0;

unsigned long Cal_freq  = 1000000UL; // Calibration Frequency: 1 Mhz = 1000000 Hz
unsigned long f[MAXMODE]            = { 7074000, 7047500, 7078000, 7038600};   //Default frequency assignment   
unsigned long slot[MAXBAND][MAXMODE]={{ 1840000, 1840000, 1842000, 1836600},   //80m [0]
                                      { 3573000, 3575000, 3578000, 3568600},   //80m [1]
                                      { 7074000, 7047500, 7078000, 7038600},   //40m [2]
                                      {10136000,10140000,10130000,10138700},   //30m [3]
                                      {14074000,14080000,14078000,14095600},   //20m [4]
                                      {18100000,18104000,18104000,18104600},   //17m [5]
                                      {21074000,21140000,21078000,21094600},   //15m [6]                           
                                      {24915000,24915000,24922000,24924600},   //12m [7] FT4 equal to FT8                           
                                      {28074000,28074000,28078000,28124600},   //10m [8]                           
                                      {50310000,50310000,50318000,50293000}};  //6m  [9]          

#ifdef CW
#define CWSHIFT         600
unsigned long qrp[MAXBAND]          = { 1810000, 3560000, 7030000,10106000,14060000,18096000,21060000,24906000,28060000,50060000};
unsigned long shift                 = CWSHIFT;
#endif //CW
                                                       
unsigned long freq      = f[Band_slot]; 
uint8_t  LED[4] ={FT8,FT4,JS8,WSPR};

#define INT0 0
#define INT1 1

#ifdef CW
#define SP   0B00000001    //simple push flag
#define PL   0B00000010    //long push flag

uint8_t       button[2]={0,0};
unsigned long downTimer[2]={0,0};
#endif //CW

//**********************************[CW mode implementation]***************************************
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
/*-----------------------------------------------------------------------------------------------------*
 *                                     CAT SubSystem                                                   *
 * cloned from uSDX (QCX-SSB) firmware                                                                 *                                    
 * adaptations and extensions by P.E.Colla (LU7DZ)                                                     *
 * ----------------------------------------------------------------------------------------------------*/
#ifdef CAT

// CAT support inspired by Charlie Morris, ZL2CTM, contribution by Alex, PE1EVX, source: http://zl2ctm.blogspot.com/2020/06/digital-modes-transceiver.html?m=1
// https://www.kenwood.com/i/products/info/amateur/ts_480/pdf/ts_480_pc.pdf
// Code excerpts from QCX-SSB by Guido (PE1NNZ)
// Mods by Pedro E. Colla(LU7DZ) 2022

void switch_RXTX(bool t);     //advance definition for compilation purposes
//*--- GETFreqA
void Command_GETFreqA()          //Get Frequency VFO (A)
{
  sprintf(hi,"FA%011ld;",freq);
  Serial.print(hi);
}

void Command_SETFreqA()          //Set Frequency VFO (A)
{
  char Catbuffer[16];
  strncpy(Catbuffer,CATcmd+2,11);
  Catbuffer[11]='\0';

  freq=(uint32_t)atol(Catbuffer);
  Command_GETFreqA();
}

void Command_GETFreqB()          //Get Frequency VFO (B) -- fallback to VFO (A) until implementation of VFOA/B pair
{
 Command_GETFreqA();    //While A/B VFO isn't implemented changes the unique VFO
}

void Command_SETFreqB()          //Set Frequency VFO (B) -- fallback to VFO (B) until implementation for VFOA/B pair
{
  Command_SETFreqA();    //While A/B VFO isnt't implemented changes the unique VFO
}

void Command_IF()               //General purpose status information command (IF), needs to be modified if RIT is implemented
{

  char txrx = (getWord(SSW,TXON)==true ? '1' : '0');
  
  sprintf(hi,"IF%011ld",freq);   //Freq
  Serial.print(hi);
  sprintf(hi,"00000");       //Empty
  Serial.print(hi);
  sprintf(hi,"+0000");      //RIT (not implemented)
  Serial.print(hi);
  sprintf(hi,"0");          //RIT OFF
  Serial.print(hi);
  sprintf(hi,"0");          //XIT OFF
  Serial.print(hi);
  sprintf(hi,"0");          //Always 0
  Serial.print(hi);
  sprintf(hi,"00");         //Memory channel 00
  Serial.print(hi);
  sprintf(hi,"%c",txrx);    //RX/TX Status
  Serial.print(hi);
  sprintf(hi,"%c",modeTS480());
  Serial.print(hi);
  sprintf(hi,"0000000;");   //Constants
  Serial.print(hi);
  
}

void Command_ST() {       //STEP command Modify when STEP is implemented
   sprintf(hi,"ST01;");
   Serial.print(hi);
}

void Command_GetMD()      //Get MODE command, only USB (2) and CW (3) are supported, 4 digital modes (mode 0,1,2,3 are mapped as USB)
{
  sprintf(hi,"MD%c;",modeTS480());
  Serial.print(hi);
}

void Command_SetMD()      //Set MODE command, only USB (2) and CW (3) are supported
{

  sprintf(hi,"MD2;");    // at this time only USB is allowed, needs to be modified when CW is added
  Serial.print(hi);
  
}


void Command_RX()
{
  switch_RXTX(false);
  Serial.print("RX0;");
}

void Command_TX()
{
  switch_RXTX(true);
  Serial.print("TX0;");
}


void Command_VX()
{
  sprintf(hi,"VX%c;",(getWord(SSW,VOX)==true ? '1' : '0'));
}

//*---- Translate mode into the TS-480 coding for mode

char modeTS480() {

  switch(mode) {
    case 0 : return '2';
    case 1 : return '2';
    case 2 : return '2';
    case 3 : return '3';
    case 4 : return '2';
    case 5 : return '2';
    case 6 : return '2';
    case 7 : return '2';
    case 8 : return '2';
    case 9 : return '2';
    default: return '2';

  }
}
void Command_AS() {

  sprintf(hi,"AS000%011ld%c;",freq,modeTS480());
  Serial.print(hi);
  return;
}
void Command_XI() {

  sprintf(hi,"XI%011ld%c0;",freq,modeTS480());
  Serial.print(hi);
  return;
}




void Command_BChange(int c) { //Change band up or down

  //Do not return anything
}
/*---------------------------------------------------------------------------------------------
 *  CAT Main command parser and dispatcher
 *---------------------------------------------------------------------------------------------*/
void analyseCATcmd()
{
  if     ((CATcmd[0] == 'F') && (CATcmd[1] == 'A') && (CATcmd[2] == ';')) {Command_GETFreqA();}
  else if((CATcmd[0] == 'F') && (CATcmd[1] == 'A') && (CATcmd[13] == ';')) Command_SETFreqA();
  else if((CATcmd[0] == 'F') && (CATcmd[1] == 'B') && (CATcmd[2] == ';')) {Command_GETFreqB();}
  else if((CATcmd[0] == 'F') && (CATcmd[1] == 'B') && (CATcmd[13] == ';')) Command_SETFreqB(); 
  else if((CATcmd[0] == 'A') && (CATcmd[1] == 'C'))                        {Serial.print("AC000;");}
  else if((CATcmd[0] == 'A') && (CATcmd[1] == 'G'))                        {Serial.print("AG0000;");}
  else if((CATcmd[0] == 'A') && (CATcmd[1] == 'N'))                        {Serial.print("AN0;");}
  else if((CATcmd[0] == 'M') && (CATcmd[1] == 'F'))                        {Serial.print("MF0;");}
  else if((CATcmd[0] == 'N') && (CATcmd[1] == 'R'))                        {Serial.print("NR0;");}
  else if((CATcmd[0] == 'P') && (CATcmd[1] == 'R'))                        {Serial.print("PR0;");}
  else if((CATcmd[0] == 'S') && (CATcmd[1] == 'U'))                        {Serial.print("SU00000000000;");}
  else if((CATcmd[0] == 'M') && (CATcmd[1] == 'G'))                        {Serial.print("MG000;");}
  else if((CATcmd[0] == 'M') && (CATcmd[1] == 'L'))                        {Serial.print("ML000;");}
  else if((CATcmd[0] == 'N') && (CATcmd[1] == 'L'))                        {Serial.print("NL000;");}
  else if((CATcmd[0] == 'O') && (CATcmd[1] == 'P'))                        {Serial.print("OP000;");}
  else if((CATcmd[0] == 'P') && (CATcmd[1] == 'B'))                        {Serial.print("PB000;");}
  else if((CATcmd[0] == 'P') && (CATcmd[1] == 'C'))                        {Serial.print("PC005;");}
  else if((CATcmd[0] == 'P') && (CATcmd[1] == 'L'))                        {Serial.print("PL000000;");}
  else if((CATcmd[0] == 'P') && (CATcmd[1] == 'A'))                        {Serial.print("PA00;");}
  else if((CATcmd[0] == 'T') && (CATcmd[1] == 'N'))                        {Serial.print("TN00;");}
  else if((CATcmd[0] == 'T') && (CATcmd[1] == 'O'))                        {Serial.print("TO0;");}
  else if((CATcmd[0] == 'T') && (CATcmd[1] == 'S'))                        {Serial.print("TS0;");}
  else if((CATcmd[0] == 'M') && (CATcmd[1] == 'R'))                        {Serial.print("MR00000000000000000000000000000000000000000000000;");}
  else if((CATcmd[0] == 'A') && (CATcmd[1] == 'S'))                        Command_AS();
  else if((CATcmd[0] == 'S') && (CATcmd[1] == 'T'))                        Command_ST();      //Step when implemented
  else if((CATcmd[0] == 'X') && (CATcmd[1] == 'I'))                        Command_XI();      //Step when implemented
  else if((CATcmd[0] == 'B') && (CATcmd[1] == 'C'))                        {Serial.print("BC0;");}
  else if((CATcmd[0] == 'N') && (CATcmd[1] == 'B'))                        {Serial.print("NB0;");}
  else if((CATcmd[0] == 'B') && (CATcmd[1] == 'S'))                        {Serial.print("BS0;");}
  else if((CATcmd[0] == 'B') && (CATcmd[1] == 'D'))                        Command_BChange(-1);
  else if((CATcmd[0] == 'B') && (CATcmd[1] == 'U'))                        Command_BChange(+1);
  else if((CATcmd[0] == 'B') && (CATcmd[1] == 'Y'))                        {Serial.print("BY0;");}
  else if((CATcmd[0] == 'X') && (CATcmd[1] == 'T'))                        {Serial.print("XT0;");}
  else if((CATcmd[0] == 'X') && (CATcmd[1] == 'O'))                        {Serial.print("XO000000000000;");}
  else if((CATcmd[0] == 'C') && (CATcmd[1] == 'A'))                        {Serial.print("CA0;");}
  else if((CATcmd[0] == 'C') && (CATcmd[1] == 'H'))                        {}
  else if((CATcmd[0] == 'S') && (CATcmd[1] == 'V'))                        {}
  else if((CATcmd[0] == 'M') && (CATcmd[1] == 'W'))                        {}
  else if((CATcmd[0] == 'Q') && (CATcmd[1] == 'I'))                        {}
  else if((CATcmd[0] == 'R') && (CATcmd[1] == 'C'))                        {}
  else if((CATcmd[0] == 'S') && (CATcmd[1] == 'R'))                        {}
  else if((CATcmd[0] == 'U') && (CATcmd[1] == 'P'))                        {}
  else if((CATcmd[0] == 'V') && (CATcmd[1] == 'R'))                        {}
  else if((CATcmd[0] == 'V') && (CATcmd[1] == 'V'))                        {}
  else if((CATcmd[0] == 'D') && (CATcmd[1] == 'N'))                        {}
  else if((CATcmd[0] == 'V') && (CATcmd[1] == 'D'))                        {Serial.print("VD0100;");}  //Review regarding VOX
  else if((CATcmd[0] == 'V') && (CATcmd[1] == 'X'))                        {Serial.print("VX1;");}  //Review regarding VOX
  else if((CATcmd[0] == 'C') && (CATcmd[1] == 'N'))                        {Serial.print("CN00;");}  
  else if((CATcmd[0] == 'V') && (CATcmd[1] == 'G'))                        {Serial.print("VC009;");}  
  else if((CATcmd[0] == 'Q') && (CATcmd[1] == 'R'))                        {Serial.print("QR00;");}  
  else if((CATcmd[0] == 'S') && (CATcmd[1] == 'H'))                        {Serial.print("SH00;");}  
  else if((CATcmd[0] == 'S') && (CATcmd[1] == 'S'))                        {Serial.print("SS0000000000000;");}  
  else if((CATcmd[0] == 'R') && (CATcmd[1] == 'L'))                        {Serial.print("RL00;");}  
  else if((CATcmd[0] == 'S') && (CATcmd[1] == 'L'))                        {Serial.print("SL00;");}  
  else if((CATcmd[0] == 'C') && (CATcmd[1] == 'T'))                        {Serial.print("CT0;");}  
  else if((CATcmd[0] == 'D') && (CATcmd[1] == 'L'))                        {Serial.print("DL000;");}  
  else if((CATcmd[0] == 'S') && (CATcmd[1] == 'D'))                        {Serial.print("SD0000;");}  
  else if((CATcmd[0] == 'S') && (CATcmd[1] == 'M'))                        {Serial.print("SM00000;");}  
  else if((CATcmd[0] == 'S') && (CATcmd[1] == 'Q'))                        {Serial.print("SQ0000;");}  
  else if((CATcmd[0] == 'R') && (CATcmd[1] == 'G'))                        {Serial.print("RG000;");}  
  else if((CATcmd[0] == 'R') && (CATcmd[1] == 'A'))                        {Serial.print("RA0000;");}  
  else if((CATcmd[0] == 'R') && (CATcmd[1] == 'M'))                        {Serial.print("RM00000;");}  
  else if((CATcmd[0] == 'R') && (CATcmd[1] == 'S'))                        {Serial.print("RS0;");}  
  else if((CATcmd[0] == 'R') && (CATcmd[1] == 'T'))                        {Serial.print("RT0;");}      //To be reviewed when RIT is implemented
  else if((CATcmd[0] == 'R') && (CATcmd[1] == 'X'))                        {Serial.print("RX0;");}      //To be reviewed when RIT is implemented
  else if((CATcmd[0] == 'S') && (CATcmd[1] == 'C'))                        {Serial.print("SC00;");}      //To be reviewed when RIT is implemented
  else if((CATcmd[0] == 'I') && (CATcmd[1] == 'S'))                        {Serial.print("IS00000;");}  
  else if((CATcmd[0] == 'K') && (CATcmd[1] == 'S'))                        {Serial.print("KS000;");}  
  else if((CATcmd[0] == 'T') && (CATcmd[1] == 'Y'))                        {Serial.print("TY000;");}  
  else if((CATcmd[0] == 'K') && (CATcmd[1] == 'Y'))                        {Serial.print("KY1;");}  
  else if((CATcmd[0] == 'R') && (CATcmd[1] == 'U'))                        {Serial.print("RU1;");}  
  else if((CATcmd[0] == 'U') && (CATcmd[1] == 'L'))                        {Serial.print("UL0;");}  
  else if((CATcmd[0] == 'L') && (CATcmd[1] == 'K'))                        {Serial.print("LK00;");}  
  else if((CATcmd[0] == 'L') && (CATcmd[1] == 'M'))                        {Serial.print("LM00000;");}  
  else if((CATcmd[0] == 'M') && (CATcmd[1] == 'C'))                        {Serial.print("MC000;");}   
  else if((CATcmd[0] == 'E') && (CATcmd[1] == 'X'))                        {Serial.print("EX000000000;");}  
  else if((CATcmd[0] == 'I') && (CATcmd[1] == 'F') && (CATcmd[2] == ';'))  {Command_IF();}
  else if((CATcmd[0] == 'I') && (CATcmd[1] == 'D') && (CATcmd[2] == ';'))  {Serial.print("ID020;");}
  else if((CATcmd[0] == 'P') && (CATcmd[1] == 'S'))                        {Serial.print("PS1;");}
  else if((CATcmd[0] == 'R') && (CATcmd[1] == 'D'))                        {Serial.print("RD1;");}
  else if((CATcmd[0] == 'A') && (CATcmd[1] == 'I'))                        {Serial.print("AI0;");}
  else if((CATcmd[0] == 'M') && (CATcmd[1] == 'D') && (CATcmd[2] == ';'))  Command_GetMD();
  else if((CATcmd[0] == 'M') && (CATcmd[1] == 'D') && (CATcmd[3] == ';'))  Command_SetMD();
  else if((CATcmd[0] == 'R') && (CATcmd[1] == 'X'))                        {Command_RX();}
  else if((CATcmd[0] == 'T') && (CATcmd[1] == 'X'))                        {Command_TX();}
  else if((CATcmd[0] == 'V') && (CATcmd[1] == 'X'))                        {Command_VX();} 
  else if((CATcmd[0] == 'X') && (CATcmd[1] == 'T'))                       {Serial.print("XT1;");}
  else if((CATcmd[0] == 'F') && (CATcmd[1] == 'L'))                       {Serial.print("FL0;");}  // 
  else if((CATcmd[0] == 'G') && (CATcmd[1] == 'T'))                       {Serial.print("GT000;");}  // 
  else if((CATcmd[0] == 'F') && (CATcmd[1] == 'R'))                       {Serial.print("FR0;");}  // Modify when VFO A/B is implemented and split is used
  else if((CATcmd[0] == 'F') && (CATcmd[1] == 'T'))                       {Serial.print("FT0;");}  // Modify when VFO A/B is implemented and split is used
  else if((CATcmd[0] == 'F') && (CATcmd[1] == 'S'))                       {Serial.print("FS0;");}  // Modify when VFO A/B is implemented
  else if((CATcmd[0] == 'F') && (CATcmd[1] == 'W'))                       {Serial.print("FW0000;");}  // Modify when VFO A/B is implemented
  else                                                                    Serial.print("?;");
  
}
/*-----------------------------------------------------------------*
 * serialEvent
 * Process serialEvent
 *-----------------------------------------------------------------*/
volatile uint8_t cat_ptr = 0;
void serialEvent(){
  

  if(!Serial.available()){
    return;
  }
    
  rxend_event = millis() + 10;  
  char data = Serial.read();

  if (data=="\n" || data=="\r" || data==0x0a) {
       return;  
  }

#ifdef ECHO
  Serial.print(data);
#endif
    
  CATcmd[cat_ptr++] = data;
    
  if(data == ';'){
      
      CATcmd[cat_ptr] = '\0'; // terminate the array
      cat_ptr = 0;            // reset for next CAT command
      analyseCATcmd();
      delay(10);
  
  } else {
      if(cat_ptr > (CATCMD_SIZE - 1)){
         Serial.print("E;"); 
         cat_ptr = 0;  // overrun       
      }
  } 
}

#endif //CAT
/*--------------------------------------------------------------------------------------------*
 * Initialize DDS SI5351 object
 *--------------------------------------------------------------------------------------------*/
Si5351 si5351;

void setup_si5351() {
//------------------------------- SET SI5351 VFO -----------------------------------  
// The crystal load value needs to match in order to have an accurate calibration
//---------------------------------------------------------------------------------
#ifdef DEBUG
  Serial.print("setup_si5351()\n");
#endif //DEBUG
 
#define XT_CAL_F   33000 
long cal = XT_CAL_F;

  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);// SET For Max Power
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA);// Set for reduced power for RX 

  
}

/*---------------------------------------------------------------------------------------------*
 * Switch between RX and TX
 *---------------------------------------------------------------------------------------------*/
void switch_RXTX(bool t) {  //t=False (RX) : t=True (TX)

#ifdef DEBUG
  if (t != getWord(SSW,TXON)) {
      sprintf(hi,"switch_RXTX(%s)\n",BOOL2CHAR(t));
      Serial.print(hi);
  } 
#endif
  
  if (t) {    //Set to TX

/*-----------------------------------*
 *               TX                  *
 *-----------------------------------*/
     digitalWrite(RX,LOW);
     si5351.output_enable(SI5351_CLK1, 0);   //RX off
     si5351.set_freq(freq*100ULL, SI5351_CLK0);
     si5351.output_enable(SI5351_CLK0, 1);   // TX on
     digitalWrite(TX,HIGH);
     setWord(&SSW,TXON,HIGH);

     
     return;
  }

/*------------------------------------*
 *                RX                  *
 *------------------------------------*/
    digitalWrite(RX,HIGH);
    si5351.output_enable(SI5351_CLK0, 0);   //TX off
    si5351.set_freq(freq*100ULL, SI5351_CLK1);
    si5351.output_enable(SI5351_CLK1, 1);   //RX on
    digitalWrite(TX,0); 
    setWord(&SSW,TXON,LOW);
    setWord(&SSW,VOX,LOW);

/*---------------------------------------------------------*
 * set to master frequency                                 *
 *---------------------------------------------------------*/
  
}

/*-----------------------------------------------------------------------------------*
 * LED management functions                                                          *
 *-----------------------------------------------------------------------------------*/
/*----- 
 * Turn off all LEDs
 */
void resetLED() {               //Turn-off all LEDs

   digitalWrite(WSPR, LOW); 
   digitalWrite(JS8, LOW); 
   digitalWrite(FT4, LOW); 
   digitalWrite(FT8, LOW); 

#ifdef DEBUG
   Serial.println("resetLED() Ok");
#endif //DEBUG   

 
}

/*-----
 * Set a particular LED ON
 */
void setLED(uint8_t LEDpin) {      //Turn-on LED {pin}
   
   resetLED();
   digitalWrite(LEDpin,HIGH);

#ifdef DEBUG
   sprintf(hi,"setLED(%d)\n",LEDpin);
   Serial.print(hi);
#endif //DEBUG

}

/*-------
 * Blink a given LED
 */
void blinkLED(uint8_t LEDpin) {    //Blink 3 times LED {pin}

#ifdef DEBUG
   sprintf(hi,"blinkLED(%d)\n",LEDpin);
   Serial.print(hi);
#endif //DEBUG
   
   uint8_t n=(MAXBLINK-1);

   while (n>0) {
       digitalWrite(LEDpin,HIGH);
       delay(BDLY);
       digitalWrite(LEDpin,LOW);
       delay(BDLY);
       n--;

#ifdef WDT       
       wdt_reset();
#endif //WDT       
   }
}

/*-----
 * LED on callibration mode
 */
void callibrateLED(){           //Set callibration mode

   digitalWrite(WSPR, HIGH); 
   digitalWrite(FT8, HIGH);
   delay(DELAY_CAL);        
}
/*==========================================================================================*/
/*----------------------------------------------------------*
 * Mode assign                                              *
 *----------------------------------------------------------*/
void Mode_assign(){
      
   freq=f[mode];
   setLED(LED[mode]);

/*--------------------------------------------*   
 * Update master frequency here               *
 *--------------------------------------------*/

#ifdef EE
   tout=millis();
   setWord(&SSW,SAVEEE,true);
#endif //EE

#ifdef DEBUG
     sprintf(hi,"Mode_assign() mode(%d) f(%ld)\n",mode,f[mode]);
     Serial.print(hi);
#endif     
}
/*----------------------------------------------------------*
 * Frequency assign (band dependant)                        *
 *----------------------------------------------------------*/
void Freq_assign(){

    uint8_t b;
    uint16_t Band=Bands[Band_slot];
    
    switch(Band) {
      case 160: b=0;break;
      case 80 : b=1;break;
      case 40 : b=2;break;
      case 30 : b=3;break;
      case 20 : b=4;break;
      case 17 : b=5;break;
      case 15 : b=6;break;
      case 12 : b=7;break;
      case 10 : b=8;break;
      case 6  : b=9;break;
      default : b=2;break;     //40m is the default
    }
    for (int i=0;i<MAXMODE;i++) {
      f[i]=slot[b][i];

#ifdef WDT      
      wdt_reset();    //Although quick don't allow loops to occur without a wdt_reset()
#endif //WDT      

    }


/*---------------------------------------*          
 * Update master frequency here          *
 *---------------------------------------*/
    freq=f[Band_slot];

#ifdef EE
    tout=millis();
    setWord(&SSW,SAVEEE,true);
#endif //EE

#ifdef DEBUG
    sprintf(hi,"Freq_assign(): B(%d) b[%d] m[%d] slot[%d] f[0]=%ld f[1]=%ld f[2]=%ld f[3]=%ld\n",Band,b,mode,Band_slot,f[0],f[1],f[2],f[3]);
    Serial.print(hi);
#endif
}

/*----------------------------------------------------------*
 * Band assignment based on selected slot                   *
 *----------------------------------------------------------*/

void Band_assign(){

 resetLED();
 blinkLED(LED[3-Band_slot]);
 delay(DELAY_WAIT); 
 Freq_assign();
 Mode_assign();
 
#ifdef DEBUG
 sprintf(hi,"Band_assign()m(%d) slot(%d)\n",mode,Band_slot);
 Serial.print(hi);
#endif //DEBUG
  
}
/*----------------------------------------------------------*
 * Manually turn TX while pressed                           *
 *----------------------------------------------------------*/
bool getTXSW();
void ManualTX(){
    
    switch_RXTX(HIGH);
    while(getTXSW()==LOW) {

#ifdef WDT      
      wdt_reset();
#endif //WDT
                
    }
    switch_RXTX(LOW);
}

bool getSwitchPL(uint8_t pin) {

  if (pin == 2) {
     if (getWord(SSW,UPPUSH) == true && getWord(button[0],PL) == true) {
        setWord(&button[0],PL,false);
        setWord(&SSW,UPPUSH,false);
        return LOW;
     }
     return HIGH;
  }

  if (pin == 3) {
     if (getWord(SSW,DNPUSH) == true && getWord(button[1],PL) == true) {
        setWord(&button[1],PL,false);
        setWord(&SSW,DNPUSH,false);
        return LOW;
     }
     return HIGH;
  }

}
/*----------------------------------------------------------*
 * get value for a digital pin and return after debouncing  *
 *----------------------------------------------------------*/
bool getSwitch(uint8_t pin) {

    if (pin ==2) {

       if (getWord(SSW,UPPUSH)==true) {
          if (getWord(button[0],SP)==true) {
             setWord(&button[0],SP,false);
             setWord(&SSW,UPPUSH,false);
             return LOW;
          }
       }
       return HIGH;
    }

    if (pin ==3) {

       if (getWord(SSW,DNPUSH)==true) {
          if (getWord(button[1],SP)==true) {
             setWord(&button[1],SP,false);
             setWord(&SSW,DNPUSH,false);
             return LOW;
          }
       }
       return HIGH;
    }
    bool state=digitalRead(pin);
 
    if (state==HIGH) {
       delay(100);
       state=digitalRead(pin);
       if (state==HIGH) {               
          return HIGH;
       }
    }
           
    return LOW;        
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
/*----------------------------------------------------------*
 * read TXSW switch
 *----------------------------------------------------------*/
bool getTXSW() {
  
    return getSwitch(TXSW);
}
/*----------------------------------------------------------*
 * Select band to operate
 *----------------------------------------------------------*/
void Band_Select(){
  
   resetLED();

#ifdef DEBUG
   sprintf(hi,"Band_Select() slot(%d) LED(%d)\n",Band_slot,LED[3-Band_slot]);
   Serial.print(hi);
#endif //DEBUG
   
   blinkLED(LED[3-Band_slot]);
   setLED(LED[3-Band_slot]);
   
   while (true) {
#ifdef WDT
      wdt_reset();
#endif //WDT      
      
      bool upButton   = getUPSSW();
      bool downButton = getDOWNSSW();
      bool txButton   = getTXSW();
          
      if ((upButton == LOW) && (downButton == HIGH)) {
          Band_slot=(Band_slot-1)%4;
          setLED(LED[3-Band_slot]);

#ifdef DEBUG
          sprintf(hi,"Band_Select() slot(%d)\n",Band_slot);
          Serial.print(hi);
#endif
          
      } 
   
      if ((upButton == HIGH) && (downButton == LOW)) {
         Band_slot=(Band_slot+1)%4;
         setLED(LED[3-Band_slot]);

#ifdef DEBUG
         sprintf(hi,"Band_Select() slot(%d)\n",Band_slot);
         Serial.print(hi);
#endif

      }                                               
      if (txButton == LOW) {
         digitalWrite(TX,0);
#ifdef EE
      tout=millis();
      setWord(&SSW,SAVEEE,true);
#endif //EE

      Band_assign();
/*------------------------------------------*
 * Update master frequency                  *
 *------------------------------------------*/
         
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
   
  while (n>0) {

#ifdef WDT
     wdt_reset();
#endif //WDT
         
     callibrateLED();
     n--;

#ifdef WDT
  wdt_reset();
#endif //WDT
  }

#ifdef EE  
  EEPROM.get(EEPROM_CAL,cal_factor);
#endif //EEPROM
  
  while (true) {

#ifdef WDT
     wdt_reset();
#endif //WDT
     
     if (getUPSSW() == LOW) {
        cal_factor = cal_factor - 100;

#ifdef EE
        EEPROM.put(EEPROM_CAL, cal_factor); 
#endif //EEPROM
        
        si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
    
  // Set Calibration CLK output
  
        si5351.set_freq(Cal_freq * 100, SI5351_CLK0);
        si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA); // Set for lower power for calibration
        si5351.set_clock_pwr(SI5351_CLK0, 1); // Enable the clock for calibration
     } 
   

     if (getDOWNSSW() == LOW) {
        cal_factor = cal_factor + 100;

#ifdef EE
        EEPROM.put(EEPROM_CAL, cal_factor);    
#endif //EEPROM
        
        si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);

  // Set Calbration Clock output
           
        si5351.set_freq(Cal_freq * 100ULL, SI5351_CLK0);
        si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA); // Set for lower power for Calibration
        si5351.set_clock_pwr(SI5351_CLK0, 1); // Enable clock2 
     } 

  }

#ifdef DEBUG
  Serial.print("Calibration() end\n");
#endif

}
#ifdef EE
/*------------------------------------------------------------------------------*
 * updateEEPROM                                                                 *
 * selectively sets values into EEPROM                                          *
 *------------------------------------------------------------------------------*/
void updateEEPROM() {

uint16_t save=EEPROM_SAVE;

   EEPROM.put(EEPROM_TEMP,save);
   EEPROM.put(EEPROM_CAL,cal_factor);
   EEPROM.put(EEPROM_MODE,mode);
   EEPROM.put(EEPROM_BAND,Band_slot);

#ifdef DEBUG
    sprintf(hi,"updateEEPROM()save(%d) cal(%d) m(%d) slot(%d)\n",save,cal_factor,mode,Band_slot);
    Serial.print(hi);
#endif //DEBUG 

    setWord(&SSW,SAVEEE,false);

}
#endif //EE

/*----------------------------------------------------------*
 * Initialization function from EEPROM                      *
 *----------------------------------------------------------*/
void INIT(){

#ifdef EE

 uint16_t temp;
 uint16_t save=EEPROM_SAVE;

 
 EEPROM.get(EEPROM_TEMP,temp);
 
 #ifdef EEPROM_CLR
   temp=-1;
 #endif //EEPROM_CLR  
 
 if (temp != save){

    updateEEPROM();
    
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

#ifdef DEBUG
   sprintf(hi,"INIT() EEPROM Read cal(%d) m(%d) slot(%d)\n",cal_factor,mode,Band_slot);
   Serial.print(hi);
#endif //DEBUG
 }  

#endif // EE
  
 Band_assign();
 Freq_assign();
 Mode_assign();
 switch_RXTX(LOW);   //Turn-off transmitter, establish RX LO

#ifdef DEBUG 
 Serial.println("INIT(): Ok");
#endif //DEBUG 
}
/*--------------------------------------------------------------------------*
 * definePinOut
 * isolate pin definition on a board conditional procedure out of the main
 * setup() flow
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

#ifdef DEBUG
   Serial.println("definePinOut()");
#endif //DEBUG      
}


#ifdef CW

/*---
 * ISR_Button()
 * Handle push button interrupt
 */
bool ISR_Button(int buttonPin) {

#ifdef DEBUG
  Serial.println("ISR_Button()");
#endif 
    
  int buttonState = digitalRead(buttonPin);
  if (buttonState == LOW) {
     downTimer[buttonPin-2]=millis();
     setWord(&button[buttonPin-2],SP,false);
     setWord(&button[buttonPin-2],PL,false);
  } else {
     long int timerDown=millis()-downTimer[buttonPin-2];
     if (timerDown <  20) { return; }
     if (timerDown < 500) {         
        setWord(&button[buttonPin-2],SP,true);
#ifdef DEBUG
        sprintf(hi,"ISR_Button(%d):SP button[%d]=%d\n",buttonPin,buttonPin-2,button[buttonPin-2]);
        Serial.print(hi);
#endif //DEBUG
        return true;
     } else {

        setWord(&button[buttonPin-2],PL,true);
#ifdef DEBUG        
        sprintf(hi,"ISR_Button(%d):SP button[%d]=%d\n",buttonPin,buttonPin-2,button[buttonPin-2]);
#endif //DEBUG 
        return true;       
     }
  }
  return false;
}
void ISR_D2() {

  setWord(&SSW,UPPUSH,ISR_Button(UP));
 
}
void ISR_D3() {

  setWord(&SSW,DNPUSH,ISR_Button(DOWN));
}
#endif //CW
//*************************************[ SETUP FUNCTION ]************************************** 
void setup()
{

#ifdef DEBUG
   Serial.begin(BAUD_DEBUG);
   sprintf(hi,"ADX Firmware V(%s)\n",VERSION);
   Serial.print(hi);
#endif //DEBUG

#ifdef CAT
   Serial.begin(BAUD_CAT);
#endif //CAT   

   definePinOut();

#ifdef CW
   attachInterrupt(INT0,ISR_D2,CHANGE);
   attachInterrupt(INT1,ISR_D3,CHANGE);
   Serial.println("Push Interrupts ok");
#endif //CW

   setup_si5351();   
   INIT();

   if ( getDOWNSSW() == LOW ) {
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
  Serial.print("setup(): Timer1 set\n");
#endif 
  
  switch_RXTX(LOW);
  Mode_assign(); 

#ifdef WDT
  wdt_disable();
  wdt_enable(WDTO_8S);
#endif //WDT
}
//*=*=*=*=*=*=*=*=*=*=*=*=*=[ END OF SETUP FUNCTION ]*=*=*=*=*=*=*=*=*=*=*=*=
/*---------------------------------------------------------------------------*
 *  checkMode
 *  manage change in mode during run-time
 *---------------------------------------------------------------------------*/
void checkMode() {
  
bool upButton     = getUPSSW();
bool downButton   = getDOWNSSW();
bool txButton     = getTXSW();
bool upButtonPL   = getSwitchPL(UP);
bool downButtonPL = getSwitchPL(DOWN);

#ifdef CW
  if (upButtonPL == LOW && getWord(SSW,CWMODE)==false) {
     setWord(&SSW,CWMODE,true);

#ifdef DEBUG     
     Serial.println("checkMode(): CW+");
#endif //DEBUG
     
  }

  if (downButtonPL == LOW && getWord(SSW,CWMODE)== true) {
     setWord(&SSW,CWMODE,false);
     
#ifdef DEBUG     
     Serial.println("checkMode(): CW-");
#endif //DEBUG     
  }
#endif //CW  


  if ((upButton == LOW)&&(downButton == LOW)&&(getWord(SSW,TXON)==false)) {

#ifdef DEBUG
      Serial.println("checkMode()U+D");
#endif //DEBUG         

     Band_Select();
  }

  if ((upButton == LOW)&&(downButton == HIGH)&&(getWord(SSW,TXON)==false)) {
      mode=(mode-1)%4;

#ifdef DEBUG
      sprintf(hi,"checkMode(): m+(%d)\n",mode);
      Serial.print(hi);
#endif //DEBUG         

#ifdef EE
      EEPROM.put(EEPROM_MODE, mode); 
#endif //EEPROM     

      Mode_assign();
  
  } 
   

  if ((upButton == HIGH) && (downButton == LOW)&&(getWord(SSW,TXON)==false)) {
      mode=(mode+1)%4;

#ifdef DEBUG
      sprintf(hi,"checkMode(): m-(%d)\n",mode);
      Serial.print(hi);
#endif //DEBUG         

#ifdef EE
      tout=millis();
      setWord(&SSW,SAVEEE,true);
#endif //EE Avoid the tear and wear of the EEPROM because of successive changes
     
      Mode_assign();
  } 


if ((txButton == LOW) && (getWord(SSW,TXON)==false)) {

#ifdef DEBUG
     sprintf(hi,"checkMode(): TX+\n");
     Serial.print(hi);
#endif //DEBUG         

   //Mode_assign();
   ManualTX();

 }
 
}
/*---------------------------------------------------------------------------------*
 * keepAlive()
 * Reference function for debugging purposes, called once per loop()
 *---------------------------------------------------------------------------------*/
void keepAlive() {

#ifdef DEBUG

   /* Code here */

#endif //DEBUG
   
}
//***************************[ Main LOOP Function ]**************************
//
//
//***************************************************************************
void loop()
{  

//*--- Debug hook
  keepAlive();

//*--- changes in mode  
  checkMode();


#ifdef EE
//*--- if EEPROM enabled check if timeout to write has been elapsed
  if((millis()-tout)>EEPROM_TOUT && getWord(SSW,SAVEEE)==true ) {
     updateEEPROM();
  }
#endif //EEPROM

#ifdef CAT 
//*--- if CAT enabled check for serial events
  serialEvent();
#endif //CAT

#ifdef WDT
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
uint16_t n = VOX_MAXTRY;

 setWord(&SSW,VOX,false);
 while (n>0){                                 //Iterate up to 10 times looking for signal to transmit

#ifdef WDT
    wdt_reset();
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
       
#ifdef DEBUG
       tx++;
       if (tx>1200) {          
          tx=0;
          sprintf(hi,"freq(%ld) f(%ld)\n",codefreq,freq);
          Serial.print(hi);
       }
#endif       
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
 switch_RXTX(LOW);
 setWord(&SSW,VOX,false);
 setWord(&SSW,TXON,false);

#ifdef WDT
 wdt_reset();
#endif //WDT     

}

//*********************[ END OF MAIN LOOP FUNCTION ]*************************
//********************************[ END OF FIRMWARE ]*************************************
