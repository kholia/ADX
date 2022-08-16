#include <Arduino.h>
#include "pdx_common.h"

#ifdef TS480
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*               DEFINITIONS SPECIFIC TO TS480 CAT PROTOCOL                                    *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
   #define CATCMD_SIZE          18  
   volatile char    CATcmd[CATCMD_SIZE];
   const int        BUFFER_SIZE = CATCMD_SIZE;
   char             buf[BUFFER_SIZE];
   
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                   TS480 CAT PROTOCOL SUBSYSTEM                                              *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
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
volatile uint16_t cat_ptr = 0;
volatile char serialBuffer[CATCMD_SIZE];

#ifdef TS480
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
   _INFOLIST("%s CAT received buffer=%s len=%d\n",__func__,serialBuffer,rc);
#endif //DEBUG  

  if (strcmp((const char*)serialBuffer,strCmd)==0) { //coincidence

#ifdef DEBUG
     _INFOLIST("%s Hit RX;ID; string\n",__func__);
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
       _INFOLIST("%s data=%c CATcmd[%d]=%c\n",__func__,data,i,CATcmd[i]);
#endif //DEBUG

       if(data == ';'){      
         CATcmd[cat_ptr] = '\0'; // terminate the array
         cat_ptr = 0;            // reset for next CAT command

#ifdef DEBUG
        _INFOLIST("%s() cmd(%s)\n",__func__,CATcmd);
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
#endif //TS480
