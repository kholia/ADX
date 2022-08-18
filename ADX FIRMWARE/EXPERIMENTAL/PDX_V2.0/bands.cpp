#include <Arduino.h>
#include "pdx_common.h"

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                             BAND SELECT CONFIGURATION                                       *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
/*
  ADX can support up to 4 bands on board. Those 4 bands needs to be assigned to Band1 ... Band4 from supported 6 bands.
  To change bands press SW1 and SW2 simultaneously. Band LED will flash 3 times briefly and stay lit for the stored band. also TX LED will be lit to indicate
  that Band select mode is active. Now change band bank by pressing SW1(<---) or SW2(--->). When desired band bank is selected press TX button briefly to exit band select mode.
  Now the new selected band bank will flash 3 times and then stored mode LED will be lit.
  TX won't activate when changing bands so don't worry on pressing TX button when changing bands in band mode.
  Assign your preferred bands to B1,B2,B3 and B4
  Supported Bands are: 80m, 40m, 30m, 20m,17m, 15m
*/


#ifdef ONEBAND                                      // If defined selects a single band to avoid mistakes with PA filter
const uint16_t Bands[BANDS] = {10, 10, 10, 10};     // All bands the same (change to suit needs)
#else
const uint16_t Bands[BANDS] = {40, 30, 20, 10};     // Band1,Band2,Band3,Band4 (initial setup)
#endif // ONEBAND

/*-----------------------------------------------------------------------------------------------------*
   This is the definition of the QUAD filter board switching, this board carries 4 filters and can
   decode up to 4 bands, so by wiring each of the 4 selectors enable any 4 band group, in the standard
   configuration bands are encoded as:
               80 -- 0 --  1
               60 -- 1 --  2
               40 -- 2 --  4
               30 -- 3 --  8
               20 -- 4 -- 16
               17 -- 5 -- 32
               15 -- 6 -- 64
               10 -- 7 --128
   This position matches the position in the slot[][] array and no change is needed, a future expansion
   will allow for all HF bands + 6 meters to be coded but the QUAD board will still be able to decode
   only 8 positions so an indirection can be made. Meanwhile it's better not to touch the quads[] defs

*/
#ifdef QUAD
#define QUADMAX         8
const uint16_t quads[QUADMAX] = {80, 60, 40, 30, 20, 17, 15, 10};
#endif //QUAD

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                   QUAD BOARD MANAGEMENT                                                     *
//*this is an optional function that support the configuration of the QUAD board on band changes*
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*

#ifdef QUAD
/*====================================================================================================*/
/*                                     QUAD Board management                                          */
/*====================================================================================================*/
/*-------------------------------------------------------------------*
   band2QUAD
   Transform a band [80..10] into the QUAD number to activate a LPF
  -------------------------------------------------------------------*/
int band2QUAD(uint16_t b) {

  int q = -1;
  for (int i = 0; i < QUADMAX; i++) {
    if (quads[i] == b) {
      q = i;
      break;
    }
  }
#ifdef DEBUG
  _INFOLIST("%s band=%d quad=%d\n", __func__, b, q);
#endif //DEBUG
  return q;
}

/*-------------------------------------------------------------------*
   setQUAD
   Set the QUAD filter with the proper slot [0..3]
  -------------------------------------------------------------------*/
void setQUAD(int LPFslot) {

  uint8_t s = 0;
  s |= (1 << LPFslot);

  Wire.beginTransmission(0x20);   // I2C device address
  Wire.write(0x09);               // Address port A
  Wire.write(s);                  // Band Relay value to write
  Wire.endTransmission();
  delay(100);

#ifdef DEBUG
  _INFOLIST("%s() LPFslot=%d QUAD=%d\n", __func__, LPFslot, s);
#endif //DEBUG

}

/*-------------------------------------------------------------------*
   setupQUAD
   init the QUAD Board [0..3]
  -------------------------------------------------------------------*/
void setupQUAD() {

  Wire.begin();                   // wake up I2C bus
  Wire.beginTransmission(0x20);   // I2C device address
  Wire.write(0x00);               // IODIRA register
  Wire.write(0x00);               // set entire PORT A as output
  Wire.endTransmission();

#ifdef DEBUG
  _INFO;
#endif //DEBUG

}
#endif //QUAD

/*---------------------------------------*
   getBand
   get a band number from frequency
   (-1) is unsupported band
  ---------------------------------------*/
int getBand(uint32_t f) {

  int b = -1;
  if (f >= 3500000 && f < 4000000) {
    b = 80;
  }
  if (f >= 5350000 && f < 5367000) {
    b = 60;
  }
  if (f >= 7000000 && f < 7300000) {
    b = 40;
  }
  if (f >= 10100000 && f < 10150000) {
    b = 30;
  }
  if (f >= 14000000 && f < 14350000) {
    b = 20;
  }
  if (f >= 18068000 && f < 18168000) {
    b = 17;
  }
  if (f >= 21000000 && f < 21450000) {
    b = 15;
  }
  if (f >= 28000000 && f < 29700000) {
    b = 10;
  }

#ifdef DEBUG
  _INFOLIST("%s() f=%ld band=%d\n", __func__, f, b);
#endif //DEBUG

  return b;
}

/*------------------------------------------------------------*
   findSlot
   find the slot [0..3] on the Bands array (band slot)
  ------------------------------------------------------------*/
int findSlot(uint16_t band) {

  int s = -1;
  for (int i = 0; i < BANDS; i++) {
    if (Bands[i] == band) {
      s = i;
      break;
    }
  }
#ifdef DEBUG
  _INFOLIST("%s() band=%d slot=%d\n", __func__, band, s);
#endif //DEBUG

  return s;

}

/*-------------------------------------------------------------*
   setSlot
   set a slot consistent with the frequency, do not if not
   supported
  -------------------------------------------------------------*/
int setSlot(uint32_t f) {
  int b = getBand(f);
  if (b == -1) {
    return Band_slot;
  }
  int s = findSlot(b);

#ifdef DEBUG
  _INFOLIST("%s() f=%ld band=%d slot=%d\n", __func__, f, b, s);
#endif //DEBUG

  return s;
}

/*-----------------------------------------------------------------*
   getMode
   given the slot in the slot[][] array and the frequency returns
   the mode that should be assigned, -1 if none can be identified
  -----------------------------------------------------------------*/
int getMode(int s, uint32_t f) {
  int m = -1;
  for (int i = 0; i < MAXMODE; i++) {
    if (int32_t(slot[s][i]/1000) == int32_t(f/1000)) {
      m = i;
      break;
    }
  }

#ifdef DEBUG
  _INFOLIST("%s slot=%d f=%ld m=%d\n", __func__, s, f, m);
#endif //DEBUG

  return m;
}
