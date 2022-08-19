#include <Arduino.h>
#include "pdx_common.h"

#ifdef TS480

/*==========================================================================================*
/*                            Start of TS480 CAT support sub-system                         *
/*==========================================================================================*


/*-----------------------------------------------------------------------------------------------------*
   CAT support inspired by Charlie Morris, ZL2CTM, contribution by Alex, PE1EVX, 
   source: http://zl2ctm.blogspot.com/2020/06/digital-modes-transceiver.html?m=1
   https://www.kenwood.com/i/products/info/amateur/ts_480/pdf/ts_480_pc.pdf
   Code excerpts from QCX-SSB by Guido (PE1NNZ)
   Mods & overall adaptation to the PDX architecture by Pedro E. Colla(LU7DZ) 2022
 *----------------------------------------------------------------------------------------------------*/


//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*               DEFINITIONS SPECIFIC TO TS480 CAT PROTOCOL                                    *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*

extern unsigned long freq;
extern boolean newCATcmd;

/*---------
 * An array where to store the received CAT data
 */
extern char CATcmd[256];  

/*---------
 * Structure to translate frequency
 */
extern int freq10GHz;
extern int freq1GHz;
extern int freq100MHz;
extern int freq10MHz;
extern int freq1MHz;
extern int freq100kHz;
extern int freq10kHz;
extern int freq1kHz;
extern int freq100Hz;
extern int freq10Hz;
extern int freq1Hz;

/*--------
 * Some general purpose internal status of an FT817, mostly fake ones
 */
extern int RIT, XIT, MEM1, MEM2, VFO, SCAN, SIMPLEX, CTCSS, TONE1, TONE2;
extern int RXSTAT;
extern int TXSTAT;
extern int MODE;

/*-------------
 * Compute the frequency in the format required by the TS480 protocol
 */
unsigned long CalcFreq()
{
  unsigned long f = (
           (10000000000L * freq10GHz) +
           (1000000000L * freq1GHz) +
           (100000000L * freq100MHz) +
           (10000000L * freq10MHz) +
           (1000000L * freq1MHz) +
           (100000L * freq100kHz) +
           (10000L * freq10kHz) +
           (1000L * freq1kHz) +
           (100L * freq100Hz) +
           (10L * freq10Hz) +
           freq1Hz);
   return f;
}
/*------------------------------------------------------
 * freq2digit
 * compute individual digits out of the transceiver freq
 */
void freq2digit(long int f) {

/*
 *  11 digits to convert from 1 GHz down to 1 Hz 
 * 
 */
    freq1Hz  =               f  % 10;
    freq10Hz =           (f/10) % 10;
    freq100Hz=          (f/100) % 10;
    freq1kHz =         (f/1000) % 10;
    freq10kHz=        (f/10000) % 10;
    freq100kHz=      (f/100000) % 10;
    freq1MHz  =     (f/1000000) % 10;
    freq10MHz =    (f/10000000) % 10;
    freq100MHz=   (f/100000000) % 10;
    freq1GHz  =  (f/1000000000) % 10;
    freq10GHz = (f/10000000000) % 10;
    return;  
}

/*---------------------------------------------------------------*
 * Commands to manage TRX state (TX/RX)
 * 
 *---------------------------------------------------------------*/
/*-----
 * Turn TX off and RX on, reply RX0;
 */

void Command_RX()
{
  RXSTAT = 0;
  TXSTAT = 0;
  setWord(&SSW,CATTX,false);
  switch_RXTX(false);
  Serial.print("RX0;");

}

/*------
 * Turn TX on and RX off
 */
void Command_TX()
{
  RXSTAT = 1;
  TXSTAT = 1;
  setWord(&SSW,CATTX,true);
  switch_RXTX(true);
  Serial.print("TX0;");
}
/*------
 * Turn TX on and RX off, reply TX0;
 */
void Command_TX1()
{
  RXSTAT = 1;
  TXSTAT = 1;
  setWord(&SSW,CATTX,true);
  switch_RXTX(true);
  Serial.print("TX0;");

}
/*------
 * Turn TX on and RX off, reply TX0;
 */
void Command_TX0()
{
  RXSTAT = 1;
  TXSTAT = 1;
  setWord(&SSW,CATTX,true);
  switch_RXTX(true);
  Serial.print("TX0;");

}

/*------
 * Turn TX on and RX off, reply TX0;
 */
void Command_TX2()
{
  RXSTAT = 1;
  TXSTAT = 1;
  setWord(&SSW,CATTX,true);
  switch_RXTX(true);
  Serial.print("TX0;");
  #ifdef DEBUG
     _INFOLIST("%s TX2;->TX0;\n",__func__);
  #endif //DEBUG   

}

/*------
 * Mode change, at this point the transceiver can be set
 * as either USB (2) or CW (3). CW can be used only if 
 * enabled by configuration properties.
 */
void Command_MD()
{
#ifdef CW
  if (mode=4) {
     Serial.print("MD3;");
  } else {   
     Serial.print("MD2;");   
  }
#else
  Serial.print("MD2;");
#endif //CW  
}

/*------
 * Generic GETFreq response, valid for both FA and FB
 * PDX doesn't carry at this point a dual VFO feature
 * therefore both VFOA and VFOB operates over the 
 * unique VFO
 */
void Command_GETFreq() {

  freq2digit(freq);

  Serial.print(freq10GHz);
  Serial.print(freq1GHz);
  Serial.print(freq100MHz);
  Serial.print(freq10MHz);
  Serial.print(freq1MHz);
  Serial.print(freq100kHz);
  Serial.print(freq10kHz);
  Serial.print(freq1kHz);
  Serial.print(freq100Hz);
  Serial.print(freq10Hz);
  Serial.print(freq1Hz);
  Serial.print(";");

}

/*--------
 * Specific response header for the FA; command
 */
void Command_GETFreqA()
{
  Serial.print("FA");
  Command_GETFreq();
}
/*--------
 * Specific response header for the FB; command
 */
void Command_GETFreqB()
{
  Serial.print("FB");
  Command_GETFreq();
}
/*-------
 * Generic response to the FAx; or FBx; command
 * as there are no dual VFO both changes the main
 * VFO frequency.
 */
void Command_SETFreqB() {

  freq10GHz  = CATcmd[2]  - 48;       // convert ASCII char to int equivalent. int 0 = ASCII 48;
  freq1GHz   = CATcmd[3]  - 48;
  freq100MHz = CATcmd[4]  - 48;
  freq10MHz  = CATcmd[5]  - 48;
  freq1MHz   = CATcmd[6]  - 48;
  freq100kHz = CATcmd[7]  - 48;
  freq10kHz  = CATcmd[8]  - 48;
  freq1kHz   = CATcmd[9]  - 48;
  freq100Hz  = CATcmd[10] - 48;
  freq10Hz   = CATcmd[11] - 48;
  freq1Hz    = CATcmd[12] - 48;

  //Command_GETFreqA();               // now RSP with FA

  Serial.print("FB");
  Serial.print(freq10GHz);
  Serial.print(freq1GHz);
  Serial.print(freq100MHz);
  Serial.print(freq10MHz);
  Serial.print(freq1MHz);
  Serial.print(freq100kHz);
  Serial.print(freq10kHz);
  Serial.print(freq1kHz);
  Serial.print(freq100Hz);
  Serial.print(freq10Hz);
  Serial.print(freq1Hz);
  Serial.print(";");

  freq = CalcFreq();

}
void Command_SETFreqA()
{
  freq10GHz  = CATcmd[2]  - 48;       // convert ASCII char to int equivalent. int 0 = ASCII 48;
  freq1GHz   = CATcmd[3]  - 48;
  freq100MHz = CATcmd[4]  - 48;
  freq10MHz  = CATcmd[5]  - 48;
  freq1MHz   = CATcmd[6]  - 48;
  freq100kHz = CATcmd[7]  - 48;
  freq10kHz  = CATcmd[8]  - 48;
  freq1kHz   = CATcmd[9]  - 48;
  freq100Hz  = CATcmd[10] - 48;
  freq10Hz   = CATcmd[11] - 48;
  freq1Hz    = CATcmd[12] - 48;

  //Command_GETFreqA();               // now RSP with FA

  Serial.print("FA");
  Serial.print(freq10GHz);
  Serial.print(freq1GHz);
  Serial.print(freq100MHz);
  Serial.print(freq10MHz);
  Serial.print(freq1MHz);
  Serial.print(freq100kHz);
  Serial.print(freq10kHz);
  Serial.print(freq1kHz);
  Serial.print(freq100Hz);
  Serial.print(freq10Hz);
  Serial.print(freq1Hz);
  Serial.print(";");

  freq = CalcFreq();

}

/*---------------
 * Transceiver status command IF;
 * Build response mixing the actual transceiver status
 * and some makeup constant responses
 */ 
void Command_IF()
{
  freq2digit(freq);
  
  Serial.print("IF");
  Serial.print(freq10GHz);        // P1
  Serial.print(freq1GHz);
  Serial.print(freq100MHz);
  Serial.print(freq10MHz);
  Serial.print(freq1MHz);
  Serial.print(freq100kHz);
  Serial.print(freq10kHz);
  Serial.print(freq1kHz);
  Serial.print(freq100Hz);
  Serial.print(freq10Hz);
  Serial.print(freq1Hz);
  Serial.print("00000");          // P2 Always five 0s
  Serial.print("+0000");          // P3 RIT/XIT freq +/-9990
  Serial.print(RIT);              // P4
  Serial.print(XIT);              // P5
  Serial.print("0");              // P6 Always 0
  Serial.print(MEM1);             // P7
  Serial.print(MEM2);             //
  if (getWord(SSW,TXON) == true) {// RX State
     Serial.print(1);
  } else {
     Serial.print(0);
  }
  Serial.print((mode==4 ? 3 : 2));// P9
  Serial.print(VFO);              // P10  FR/FT 0=VFO
  Serial.print(SCAN);             // P11
  Serial.print(SIMPLEX);          // P12
  Serial.print(CTCSS);            // P13
  Serial.print(TONE1);            // P14
  Serial.print(TONE2);
  Serial.print("0");              // P15 Always 0
  Serial.print(";");
}




/*----
 * Fake commands AI,RS,ID,PS
 * Response is built based on constants
 * (not really related to the actual transceiver operational status)
 */
void Command_AI()
{
  Serial.print("AI0;");
}

void Command_PS1()
{
  Serial.print("PS1;");
}

void Command_AI0()
{
  Serial.print("AI0;");
}


void Command_RS()
{
  Serial.print("RS0;");
}

void Command_ID()
{
  Serial.print("ID020;");
}

void Command_PS()
{
  Serial.print("PS1;");
}


/*-------
 * Additional commands, the implemented commands are required at different moments
 * by WSJT-X. However, FLDigi (and other similars based on the HamLib library) might
 * throw the following commands which aren't implemented
 * SL SH AG0 FB BC EX0450000 IS PC SQ0 RG GT FT FR
 * Many of them belongs to aspects of the transceiver status which aren't and (will not)
 * be implemented, still a basic answer can be made based on constants. At this point
 * FLDigi doesn't seems to mind if an answer isn't provided so they are not implemented
 */



/*------
 * rxCATcmd
 * Receive and process CAT commands
 */
void rxCATcmd()
{
  int index = 0;
  char endMarker = ';';
  char data;                    // CAT commands are ASCII characters

  while ((Serial.available() > 0) && (newCATcmd == false))
  {
    data = Serial.read();

    if (data != endMarker)
    {
      CATcmd[index] = data;
      index++;

      if (index >= numChars)
        index = numChars - 1;   // leave space for the \0 array termination
    }
    else
    {
      CATcmd[index] = ';';      // Indicate end of command
      CATcmd[index + 1] = '\0'; // terminate the array
      index = 0;                // reset for next CAT command
      newCATcmd = true;
    }
  }
}

/*----
 * analyseCATcmd
 * analyse and craft a appropriate answer to the CAT command
 */
void analyseCATcmd()
{
  if (newCATcmd == true)
  {
    newCATcmd = false;        // reset for next CAT time, avoid re-entrancy issues

    #ifdef DEBUG
        _TRACELIST("%s CAT->%s\n",__func__,CATcmd);
    #endif //DEBUG    

    if ((CATcmd[0] == 'F') && (CATcmd[1] == 'A') && (CATcmd[2] == ';'))              // must be freq get command
      Command_GETFreqA();

    else if ((CATcmd[0] == 'F') && (CATcmd[1] == 'B') && (CATcmd[2] == ';'))        // must be freq set command
      Command_GETFreqB();

    else if ((CATcmd[0] == 'F') && (CATcmd[1] == 'B') && (CATcmd[13] == ';'))        // must be freq set command
      Command_SETFreqB();

    else if ((CATcmd[0] == 'F') && (CATcmd[1] == 'A') && (CATcmd[13] == ';'))        // must be freq set command
      Command_SETFreqA();

    else if ((CATcmd[0] == 'I') && (CATcmd[1] == 'F') && (CATcmd[2] == ';'))
      Command_IF();

    else if ((CATcmd[0] == 'I') && (CATcmd[1] == 'D') && (CATcmd[2] == ';'))
      Command_ID();

    else if ((CATcmd[0] == 'P') && (CATcmd[1] == 'S') && (CATcmd[2] == ';'))
      Command_PS();

    else if ((CATcmd[0] == 'P') && (CATcmd[1] == 'S') && (CATcmd[2] == '1'))
      Command_PS1();

    else if ((CATcmd[0] == 'A') && (CATcmd[1] == 'I') && (CATcmd[2] == ';'))
      Command_AI();

    else if ((CATcmd[0] == 'A') && (CATcmd[1] == 'I') && (CATcmd[2] == '0'))
      Command_AI0();

    else if ((CATcmd[0] == 'M') && (CATcmd[1] == 'D') && (CATcmd[2] == ';'))
      Command_MD();

    else if ((CATcmd[0] == 'R') && (CATcmd[1] == 'X') && (CATcmd[2] == ';'))
      Command_RX();

    else if ((CATcmd[0] == 'R') && (CATcmd[1] == 'X') && (CATcmd[2] == '0'))
      Command_RX();

    else if ((CATcmd[0] == 'T') && (CATcmd[1] == 'X') && (CATcmd[2] == ';'))
      Command_TX();

    else if ((CATcmd[0] == 'T') && (CATcmd[1] == 'X') && (CATcmd[2] == '1'))
      Command_TX1();

    else if ((CATcmd[0] == 'T') && (CATcmd[1] == 'X') && (CATcmd[2] == '0'))
      Command_TX0();

      
    else if ((CATcmd[0] == 'T') && (CATcmd[1] == 'X') && (CATcmd[2] == '2'))
      Command_TX2();

    else if ((CATcmd[0] == 'R') && (CATcmd[1] == 'S') && (CATcmd[2] == ';'))
      Command_RS();

    Serial.flush();       // Get ready for next command
  }
}

/*-----
 * serialEvent()
 * Event dispatcher called periodically from the main processing loop
 * this entry point is the single entry called from the outside to this
 * sub-system, and it's common for all CAT protocol implementations on
 * the ADX/PDX architecture
 * As this processing is called serially with most process, even time
 * critical ones, the fastest possible response must be provided.
 */

void serialEvent() {
  rxCATcmd();
  analyseCATcmd();

}
/*==========================================================================================*
/*                              End of TS480 CAT support sub-system                         *
/*==========================================================================================*
#endif //TS480
