#include <Arduino.h>
#include "pdx_common.h"

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                             CONFIGURATION TERMINAL SUB-SYSTEM                               *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*   
#ifdef TERMINAL
/*----------------------------------------------------------------------------------------------*
 * Instead of changing the main parameters and create a new firmware image as a way to tune     *
 * the behaviour of the program few of the main parameters can be stored on EEPROM to become    *
 * persistent across executions. Most of them can be changed with a serial terminal and stored  *
 * in EEPROM with this sub-system.                                                              *
 */

#include <stddef.h>
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
/*              Configuration Terminal Sub-system                                                                        */
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
const char *atuToken        = "atu";

#ifdef ATUCTL
const char *atu_delayToken  = "atd";
#endif //ATUCTL

const char *max_tryToken     = "vox";
const char *bounce_timeToken = "bt";
const char *short_timeToken  = "st";
const char *max_blinkToken   = "mbl";
const char *map_asciiToken   = "map";

#ifdef EE
const char *eeprom_toutToken = "eet";
const char *listToken = "list";
#endif //EE

#ifdef NTPSYNC
const char *pskToken   = "psk";
const char *ssidToken  = "ssid";
const char *dateToken  = "date";
const char *wifi_toutToken = "wtout";
#endif //NTPSYNC

const char *saveToken       = "save";
const char *quitToken       = "quit";
const char *resetToken      = "reset";
const char *helpToken       = "help";
const char *txonToken       = "txon";
const char *txoffToken      = "txoff";
const char *cal_freqToken   = "calfreq";
const char *endList         = "?";


#define BUFFER_LEN          128
#define DELIMITER           " "
#define CR                  0x0d
#define LF                  0x0a
#define BS                  \b      
#define TERMINATOR          CR

char buffer[BUFFER_LEN]     = "";
uint16_t ptr                = 0;

/*-------------------------
 * popBang
 * Parsing procedure, more like a well behaved strtok function (which I really dislike)
 */
bool popBang(char *s,char *t) {
  char delimiter=' ';
  strcpy(t,""); 
  for (int j=0;j<= strlen(s);j++) {
     if ((char)s[j]==delimiter) {
        strcpy(s,(char*)&s[j+1]);
        if (strlen(s) == 0) {
           return false;
        } else {
           return true;
        }
     }
     t[j]=s[j];
     t[j+1]=0x00;
  }
  strcpy(s,"");
  return false;
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
  Serial.println("");
  Serial.print(">");

  return;
}
#endif //EE
/*--------------
 * perform_date
 * print date and hour from system clock
 */
void perform_date() {

  time_t now = time(nullptr);
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  Serial.println();
  sprintf(hi,"%s\n>",asctime(&timeinfo));
  Serial.print(hi);

}
/*--------------
 * perform_mapAscii
 * 
 */
void perform_mapAscii() {

  Serial.println("");
  Serial.println("ASCII Map");
  
  for (int j = 0; j < 256; j++) {
    
    sprintf(hi, "%03d %02x %c\n", (byte)j,(byte)j,(char)j);
    Serial.print(hi);
  }

  Serial.println("");
  
}
/*---
   quit command
*/
void perform_quitToken () {
  
  sprintf(hi,"\r\nExiting terminal mode\n\r");
  Serial.println(hi);
  delay(200);
  resetFunc();
  return;
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

  sprintf(hi,"%s, ",bounce_timeToken);
  Serial.print(hi);

  sprintf(hi,"%s, ",max_tryToken);
  Serial.print(hi);

  sprintf(hi,"%s, ",short_timeToken);
  Serial.print(hi);

  sprintf(hi,"%s, ",max_blinkToken);
  Serial.print(hi);
/*--------
 * EEPROM specific commands
 */
#ifdef EE
  sprintf(hi,"%s, ",eeprom_toutToken);
  Serial.print(hi);
  
  sprintf(hi,"%s, ",listToken);
  Serial.print(hi);
#endif //EE

/*--------
 * ATU specific commands
 */

#ifdef ATUCTL

  sprintf(hi,"%s, ",atuToken);
  Serial.print(hi);
  
  sprintf(hi,"%s, ",atu_delayToken);
  Serial.print(hi);
  
#endif //ATUCTL
 
#ifdef NTPSYNC

   sprintf(hi,"%s, ",ssidToken);
  Serial.print(hi);
 
  sprintf(hi,"%s, ",pskToken);
  Serial.print(hi);

  sprintf(hi,"%s, ",dateToken);
  Serial.print(hi);

  sprintf(hi,"%s, ",wifi_toutToken);
  Serial.print(hi);

#endif //NTPSYNC


  sprintf(hi,"%s, ",cal_freqToken);
  Serial.print(hi);
  
  sprintf(hi,"%s, ",txonToken);
  Serial.print(hi);


  sprintf(hi,"%s, ",txoffToken);
  Serial.print(hi);
  
  sprintf(hi,"%s, ",saveToken);
  Serial.print(hi);
  
  sprintf(hi,"%s, ",quitToken);
  Serial.print(hi);

  sprintf(hi,"%s, ",resetToken);
  Serial.print(hi);

  sprintf(hi,"%s",helpToken);
  Serial.print(hi);

  sprintf(hi,"\n\r>");
  Serial.print(hi);
}
/*------------------------
 * parse a numeric configuration parameter, if the
 * conversion from the argument is 0 it's assumed an
 * invalid number (such as blanks) and the existing
 * value of the parameter returned instead, otherwise
 * it's updated.
 */

uint16_t execNumber(char *a,uint16_t* var) {
   char n[16]="";
   popBang(a,n);
   int v=atoi(n);
   if (v==0) {
      return *var;
   }
   *var=v;
   return *var;
}
uint32_t execLongNumber(char *a,uint32_t* var) {
   char n[16]="";
   popBang(a,n);
   int v=atoi(n);
   if (v==0) {
      return *var;
   }
   *var=v;
   return *var;
}

/*------------------------------
 * print a numeric configuration parameter
 */
void cli_printNumber(char *c, uint16_t rc) {

  sprintf(hi,"%s(%05d)\n\r>", c, rc);
  Serial.print(hi);

}
/*------------------------------
 * print a numeric configuration parameter
 */
void cli_printLongNumber(char *c, uint32_t rc) {

  sprintf(hi,"%s(%09ld)\n\r>", c, rc);
  Serial.print(hi);

}

/*-----------------------------
 * process a string configuration parameter
 * if the argument is empty the current content
 * is returned, otherwise the content of the
 * parameter is replaced by the argument
 */
void execString(char *a, char *s) {
  if (strlen(a)==0) return;
  strcpy(s,a);
  return;
}
/*------------------------------
 * print a string configuration parameter
 */
void cli_printString(char *c, char *s) {

  sprintf(hi,"%s(%s)\n\r>", c, s);
  Serial.print(hi);

}
/*----------------------------------
 * print an error message
 */
void cli_printError(char *c) {

  sprintf(hi,"%s(\?\?\?\?\?) unknown or invalid command, type help or ?\n\r>", c);
  Serial.print(hi);

}
void perform_Tx(bool s) {
  switch_RXTX(s);
  sprintf(hi,"\r\nTransmitter set to(%s)\r\n>", BOOL2CHAR(s));
  Serial.print(hi);
  
}
/*----------------------------------
 * Main command dispatcher
 * the parsed command is contained in the area pointed by c
 * whilst the rest of the arguments in the area pointed by a
 * depending on the command process the argument in different
 * ways
 */
void cli_execute(char *buffer) {

char cmd[128]="";
char argv[128]="";

     popBang(buffer,cmd);
     strcpy(argv,buffer);

     if (strcmp(cmd,max_tryToken)   == 0) { execNumber(argv,&vox_maxtry); cli_printNumber(cmd,vox_maxtry);  return;}
     if (strcmp(cmd,bounce_timeToken)   == 0) { execNumber(argv,&bounce_time); cli_printNumber(cmd,bounce_time);  return;}
     if (strcmp(cmd,short_timeToken)   == 0) { execNumber(argv,&short_time); cli_printNumber(cmd,short_time);  return;}
     if (strcmp(cmd,max_blinkToken)   == 0) { execNumber(argv,&max_blink); cli_printNumber(cmd,max_blink);  return;}
     if (strcmp(cmd,cal_freqToken)   == 0) { execLongNumber(argv,&Cal_freq); cli_printLongNumber(cmd,Cal_freq);  return;}
     
#ifdef EE
     if (strcmp(cmd,eeprom_toutToken)   == 0) { execNumber(argv,&eeprom_tout); cli_printNumber(cmd,eeprom_tout);  return;}
#endif //EE

#ifdef ATUCTL
     if (strcmp(cmd,atuToken)   == 0) { execNumber(argv,&atu); cli_printNumber(cmd,atu);  return;}
     if (strcmp(cmd,atu_delayToken)   == 0) { execNumber(argv,&atu_delay); cli_printNumber(cmd,atu_delay);  return;}
#endif //ATUCTL     

#ifdef NTPSYNC
     if (strcmp(cmd,ssidToken)  == 0) { execString(argv,wifi.ssid); cli_printString(cmd,wifi.ssid); return;}
     if (strcmp(cmd,pskToken)   == 0) { execString(argv,wifi.password); cli_printString(cmd,wifi.password); return;}
     if (strcmp(cmd,wifi_toutToken)   == 0) { execNumber(argv,&wifi_tout); cli_printNumber(cmd,wifi_tout); return;}
#endif //NTPSYNC

     if (strcmp(cmd,txonToken)  == 0) { perform_Tx(HIGH); return;}
     if (strcmp(cmd,txoffToken)  == 0) { perform_Tx(LOW); return;}

     if (strcmp(cmd,helpToken)  == 0) { perform_helpToken(); return;}
     if (strcmp(cmd,endList)    == 0) { perform_helpToken(); return;}
     if (strcmp(cmd,saveToken)  == 0) { perform_saveToken(); return;}
     if (strcmp(cmd,resetToken) == 0) { perform_resetToken(); return;}
     if (strcmp(cmd,listToken)  == 0) { perform_listToken(); return;}
     if (strcmp(cmd,quitToken)  == 0) { perform_quitToken(); return;}
     if (strcmp(cmd,map_asciiToken) == 0) { perform_mapAscii(); return;}
     if (strcmp(cmd,dateToken)  == 0) { perform_date(); return;}

     cli_printError(cmd);
}
/*--------------------------
 * sub-system initialization
 */
void cli_init() {


  sprintf(hi, "\n\rPDX %s build(%03d) command interpreter\n\r", VERSION, uint16_t(BUILD));
  Serial.print(hi);
  Serial.print(">");
  strcpy(buffer,"");
  setWord(&QSW,CLIOK,true);
  
}

/*----------------------------------
 * Command processor
 * Collect the command from the serial port
 * on LF parse the command line and tokenize into a command
 * followed by arguments (one to many)
 */
void cli_command() {
  
  while (getWord(QSW,CLIOK)) {
     if (Serial.available()) {
        char c = Serial.read();
        if ((byte)c==CR) {
        }
        if ((byte)c==LF) {
           cli_execute(buffer);
           ptr=0;
           strcpy(buffer,"");
        }
        if ((byte)c!=LF && (byte)c!=CR) {
           buffer[ptr++]=c;               
           buffer[ptr]=0x00;
        }   
     }
  }    
}
#endif //TERMINAL
