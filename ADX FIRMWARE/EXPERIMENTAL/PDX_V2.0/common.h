#ifndef SOME_UNIQUE_NAME_HERE
#define SOME_UNIQUE_NAME_HERE

extern uint8_t SSW;
extern unsigned long freq;

void setWord(uint8_t* SysWord, uint8_t v, bool val);
void switch_RXTX(bool t);

void serialEvent();


#define CATTX 0B00100000  // TX turned on via CAT (disable VOX)

#endif
