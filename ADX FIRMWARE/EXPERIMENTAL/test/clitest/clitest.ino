#include <stddef.h>
char hi[128];
uint16_t atu=15;
char ssid[30]="";
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
/*              Configuration Terminal Sub-system                                                                        */
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
const char *atuToken        = "atu";
const char *ssidToken       = "ssid";

#ifdef ATUCTL
const char *atu_delayToken  = "atd";
#endif //ATUCTL

const char *max_tryToken     = "vox";
const char *bounce_timeToken = "bt";
const char *short_timeToken  = "st";
const char *max_blinkToken   = "mbl";

#ifdef EE
const char *eeprom_toutToken = "eet";
const char *eeprom_listToken = "list";
#endif //EE

#ifdef NTPSYNC
const char *wifi_pskToken   = "psk";
const char *wifi_ssidToken  = "ssid";
#endif //NTPSYNC

const char *saveToken       = "save";
const char *quitToken       = "quit";
const char *resetToken      = "reset";
const char *helpToken       = "help";
const char *endList         = "?";


#define BUFFER_LEN          128
#define DELIMITER           " "
#define CR                  0x0d
#define LF                  0x0a
#define BS                  \b      
#define TERMINATOR          CR

char buffer[BUFFER_LEN]     = "";
uint16_t ptr                = 0;
//char cmd[BUFFER_LEN]        = "";
//char argv[BUFFER_LEN]       = "";



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
  
  sprintf(hi,"%s, ",eeprom_listToken);
  Serial.print(hi);
#endif //EE

/*--------
 * ATU specific commands
 */

  sprintf(hi,"%s, ",atuToken);
  Serial.print(hi);

#ifdef ATUCTL
  
  sprintf(hi,"%s, ",atu_delayToken);
  Serial.print(hi);
  
#endif //ATUCTL

  sprintf(hi,"%s, ",ssidToken);
  Serial.print(hi);
  
#ifdef NTPSYNC
  sprintf(hi,"%s, ",wifi_pskToken);
  Serial.print(hi);


  sprintf(hi,"%s, ",wifi_ssidToken);
  Serial.print(hi);
  
#endif //NTPSYNC
  
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
/*--------------------------
 * sub-system initialization
 */
void cli_init() {

     sprintf(hi,"Configuration terminal\n");
     Serial.print(hi);
     Serial.print(">>");
     strcpy(cmd,"");
     strcpy(argv,"");
     strcpy(buffer,"");
  
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
/*------------------------------
 * print a numeric configuration parameter
 */
void cli_printNumber(char *c, uint16_t rc) {

  sprintf(hi,"%s(%05d)\n\r>", c, rc);
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

     if (strcmp(cmd,atuToken) == 0) { execNumber(argv,&atu); cli_printNumber(cmd,atu);  return;}
     if (strcmp(cmd,ssidToken)== 0) { execString(argv,ssid); cli_printString(cmd,ssid); return;}
     if (strcmp(cmd,helpToken)== 0) { perform_helpToken(); return;}
     if (strcmp(cmd,endList)  == 0) { perform_helpToken(); return;}

     cli_printError(cmd);
}
/*----------------------------------
 * Command processor
 * Collect the command from the serial port
 * on LF parse the command line and tokenize into a command
 * followed by arguments (one to many)
 */
void cli_command() {
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
/*----------------------------------------
 * this is the setup() of the test program
 */
void setup() {

    Serial.begin(115200);
    while(!Serial);
    Serial.flush();
    cli_init();
    
}
/*----------------------------------------
 * this is the loop() of the test program
 */
void loop() {
  cli_command();
}
