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
  Supported Bands are: 80m, 40m, 30m, 20m,17m, 15m unless the #define ONEBAND directive is set
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
/*====================================================================================================*/
/*                            Band and frequency management                                           */
/*====================================================================================================*/
/*----------------------------------------------------------*
   Band assignment based on selected slot
   [@@@] just blink the leds since the actual frequency is
   going to be changed by calling Freq_assign()
  ----------------------------------------------------------*/
void Band_assign(bool l) {    //@@@ Change behaviour

  if (l == true) {
    resetLED();
    
    #ifdef ONEBAND
       blinkAllLED();
#ifdef DEBUG
       _INFOLIST("%s blinkAllLED()\n", __func__);
#endif //DEBUG

    #else   
       blinkLED(LED[3 - Band_slot]);
#ifdef DEBUG
       _INFOLIST("%s blinkLED(%d)\n", __func__,LED[3-Band_slot]);
#endif //DEBUG

    #endif //ONEBAND
       
    delay(DELAY_WAIT);             //This delay should be changed
  }

#ifdef DEBUG
  _INFOLIST("%s mode(%d) slot(%d) f=%ld\n", __func__, mode, Band_slot, freq);
#endif //DEBUG
}

/*------------------------------------------------------------------*
   Assign index in slot[x][] table based on the band
  ------------------------------------------------------------------*/
uint8_t band2Slot(uint16_t b) {
  uint8_t s = -1;    //@@@ To be traslated to ADX
  switch (b) {
    case  80 : {
        s = 0;
        break;
      }
    case  60 : {
        s = 1;
        break;
      }
    case  40 : {
        s = 2;
        break;
      }
    case  30 : {
        s = 3;
        break;
      }
    case  20 : {
        s = 4;
        break;
      }
    case  17 : {
        s = 5;
        break;
      }
    case  15 : {
        s = 6;
        break;
      }
    case  12 : {
        s = 7;
        break;
      }
    case  10 : {
        s = 8;
        break;
      }
  }
#ifdef DEBUG
  _INFOLIST("%s() band=%d slot=%d\n", __func__, b, s);
#endif //DEBUG

  return s;

}

/*---------------------------------------*
   getBand
   get a band number from frequency
   (-1) is unsupported band
  ---------------------------------------*/
int getBand(uint32_t fx) {
  int b = -1;
  if (fx >= 3500000 && fx < 4000000) {
    b = 80;
  }
  if (fx >= 5350000 && fx < 5367000) {
    b = 60;
  }
  if (fx >= 7000000 && fx < 7300000) {
    b = 40;
  }
  if (fx >= 10100000 && fx < 10150000) {
    b = 30;
  }
  if (fx >= 14000000 && fx < 14350000) {
    b = 20;
  }
  if (fx >= 18068000 && fx < 18168000) {
    b = 17;
  }
  if (fx >= 21000000 && fx < 21450000) {
    b = 15;
  }

  if (fx >= 24890000 && fx < 24990000) {
    b = 12;
  }

  if (fx >= 28000000 && fx < 29700000) {
    b = 10;
  }

#ifdef DEBUG
  _INFOLIST("%s() f=%ld band=%d\n", __func__, fx, b);
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

/*-------------------------------------------------------*
   resetBand
   setup the band after a valid Band_slot has been set
   @@@ must be moved into ADX
*/
void resetBand(int bs) {

#ifdef DEBUG
  _INFOLIST("%s Searching for band Band_slot=%d ok\n", __func__, bs);
#endif //DEBUG

  int b = Bands[bs];

#ifdef DEBUG
  _INFOLIST("%s Retrieved Band_slot=%d band=%d ok\n", __func__, Band_slot, b);
#endif //DEBUG

  int i = band2Slot(b);

#ifdef DEBUG
  _INFOLIST("%s Retrieved Slot=%d Band_slot=%d ok\n", __func__, i, b);
#endif //DEBUG

  setStdFreq(i);

#ifdef DEBUG
  _INFOLIST("%s Retrieved Std frequency set f[0]=%ld f[1]=%ld f[2]=%ld f[3]=%ld f[4]=%ld Slot=%d Band_slot=%d ok\n", __func__, f[0], f[1], f[2], f[3], f[4], i, b);
#endif //DEBUG

  freq = f[mode];
#ifdef DEBUG
  _INFOLIST("%s Frequency set to freq=%ld ok\n", __func__, freq);
#endif //DEBUG

  Band_assign(true);
#ifdef DEBUG
  _INFOLIST("%s Band_assign Ok freq=%ld ok\n", __func__, freq);
#endif //DEBUG

  Mode_assign();
#ifdef DEBUG
  _INFOLIST("%s Mode_assign Ok mode=%d freq=%ld ok\n", __func__, mode, freq);
#endif //DEBUG

#ifdef QUAD
  setupQUAD();
#ifdef DEBUG
  _INFOLIST("%s setupQUAD ok\n", __func__);
#endif //DEBUG
  /*---------
    Initialize the QUAD board with the default band (at slot 0)
  */
  int q = band2QUAD(b);
  if (q != -1) {
    setQUAD(q);
  }
#ifdef DEBUG
  _INFOLIST("%s setQUAD Bands[%d]=%d quadLPF=%d\n", __func__, Band_slot, b, q);
#endif //DEBUG
#endif //QUAD

#ifdef ATUCTL
  flipATU();
#endif //ATUCTL


#ifdef DEBUG
  _INFOLIST("%s Band_slot=%d Band=%d slot=%d f[0]=%ld f[1]=%ld f[2]=%ld f[3]=%ld f[4]=%ld mode=%d freq=%ld\n", __func__, bs, b, i, f[0], f[1], f[2], f[3], f[4], mode, freq);
#endif //DEBUG
}
/*-----------------------------------------------------------*
   setStdFreq
   Set all standard frequencies for the band and transfer to
   the f[] array
   @@@ To implement on ADX
*/
void setStdFreq(int k) {

  for (int i = 0; i < MAXMODE ; i++) {
    f[i] = slot[k][i];
#ifdef WDT
    wdt_reset();    //Although quick don't allow loops to occur without a wdt_reset()
#endif //WDT
  }

#ifdef DEBUG
  _INFOLIST("%s Std frequency set f[0]=%ld f[1]=%ld f[2]=%ld f[3]=%ld f[4]=%ld Band_slot=%d mode=%d ok\n", __func__, f[0], f[1], f[2], f[3], f[4], Band_slot, mode);
#endif //DEBUG

}
/*-----------------------------------------------------------------*
   getMode @@@
   given the slot in the slot[][] array and the frequency returns
   the mode that should be assigned, -1 if none can be identified
   this method shouldn't be called when operating on CW as it will
   yield a not identified mode because CW can have a continuous
   tuning space (the space to scan is MAXMODE-2 (0.1.2.3)
  -----------------------------------------------------------------*/
int getMode(int s, uint32_t fx) {

  for (int i = 0; i < MAXMODE - 1; i++) {
    if (int32_t(slot[s][i] / 1000) == int32_t(fx / 1000)) {
      #ifdef DEBUG
         _INFOLIST("%s slot=%d f=%ld mode=%d\n",__func__,s,fx,i);
      #endif //DEBUG   
      return i;
    }
  }
  #ifdef DEBUG
     _INFOLIST("%s slot=%d f=%ld mode not found\n",__func__,s,fx);
  #endif //DEBUG   

  return -1;
}

/*-----------------------------------------------------------*
   updateFreq [@@@]
   receives a frequency change proposal and perform all
   validations needed to understand if the frequency is valid,
   belongs to a supported band, what mode the frequency is for
   change the QUAD filter if enabled, pulse the ATU if enabled
   and consider the case of CW where the frequency can be
   anything.
   The original frequency change structure of ADX (Freq_assign())
   is deprecated.
   Returns 0 if successful and -1 if not
*/
int updateFreq(uint32_t fx) {

  /*-------
     What band this frequency belongs to?

  */
  int b = getBand(fx);
  if (b == -1) {
#ifdef DEBUG
    _INFOLIST("%s Invalid band Band=%d freq=%ld\n", __func__, b, fx);
#endif //DEBUG

    return b;      //invalid band and/or frequency
  }

  /*-------
     at this point b={80,.....,10} all possible supported
     frequencies, let´s see if the board has support for
     the particular band required (Bands[])
  */
  int s = findSlot(b);
  if (s == -1) {
#ifdef DEBUG
    _INFOLIST("%s Invalid band slot Band=%d Band_slot=%d freq=%ld\n", __func__, b, s, fx);
#endif //DEBUG
    return s;    //the band is valid but not supported
  }
  /*------
     at this point s=Bands[Band1,...,Band4]
     if the rest of the validations proceed this value
     will be further committed as Band_slot.
     Now we need to retrieve the key frequencies for this
     band from slot[i][*] which will be used for the four
     digital modes supported plus the QRP calling frequency
     for CW on each band
  */
  int h = band2Slot(b);
  if (h == -1) {

#ifdef DEBUG
    _INFOLIST("%s Invalid band definition Band_slot=%d Band=%d freq=%ld\n", __func__, b, h, fx);
#endif //DEBUG

    return h;   //no entry on slog[][] found
  }

  /*-------
     So far this is a valid band and supported by the
     transceiver, it's defined also on the slot[][]
     structure.
     Now the mode has to be detected, if we're in
     digital modes (mode={0..3} then the frequency
     needs to be an accepted frequency for one of the
     digital modes, this is needed because the LED on
     the board needs to be consistently lit.
  */
  int m=0;
  if (mode != 4) {  
     m = getMode(h,fx);
     if (m == -1) {

        #ifdef DEBUG
            _INFOLIST("%s Invalid mode definition slot=%d mode=%d freq=%ld\n", __func__, h, m, fx);
        #endif //DEBUG
        return m;    //this means a digital mode has not been recognized
     }
  } else {
    m=4;   //Force to stay in CW
  }
#ifdef QUAD
  /*---------
     Now, if the QUAD filter is supported, based on the
     configuration definitions by the inclusion of the
     #include QUAD 1 directive then the QUAD filter is
     seek.
  */
  int q = band2QUAD(b);
  if (q == -1) {
    
#ifdef DEBUG
      _INFOLIST("%s Invalid QUAD filter Band=%d QUAD=%d freq=%ld\n", __func__, b, q, fx);
#endif //DEBUG

     return -1;   //this would be odd, but still possible, no filter support
  }
#endif //QUAD

  /*----------
     thus far all validations has been performed, the frequency change
     is legit and can be supported, so let´s commit to it
     If a band change is detected then setup the transceiver to the new
     band.
     b contains the band {80,..,10}
     s contains the new Band_slot {0,..,3}
     h contains the index on the slot[][] structure {0,..,9}
     m contains the mode {0..3}
     q contains the QUAD filter index {0,..,3}
  */


/*---
 * Check if the band changes with the new frequency
 */
  
  if (s != Band_slot) {
     Band_slot=s;
     Band_assign(false);     //Set everything but don't waste time with LED lighting dance

#ifdef QUAD
     setQUAD(q);          //Set the filter before any TX is made
#endif //QUAD

#ifdef ATUCTL
    flipATU();           //Reset the ATU if present
#endif//
    /*-------
       Prepare the frequency set for different modes in the new band
    */
    mode=m;
    setStdFreq(h);

    /*---
     * Change the mode, primarily the LED associated to it
     */
    Mode_assign();
    /*---
     * Band_slot and mode are attributes stored in EEPROM so schedule an EEPROM
     * update
     */
     #ifdef EE
        tout = millis();
        setWord(&SSW, SAVEEE, true);
     #endif //EE

     /*---
      * Force a clock change with the new frequency
      */
     switch_RXTX(LOW);
     return 0;

  }
  /*---------------
     At this point either no band change has been made or all the setup for a band
     change has been completed. So check if additionally a mode change is needed
     unless it's CW where the mode has been set already
  */
  if (m != mode) {
     mode = m;
     Mode_assign();       //Mode change

     #ifdef EE
         tout = millis();
         setWord(&SSW, SAVEEE, true);
     #endif //EE
     
     #ifdef DEBUG
         _INFOLIST("%s Modechanged Band_slot=%d mode=%d freq=%ld\n", __func__, s, mode, fx);
     #endif //DEBUG
  }

  /*----
   * This means the change to the frequency can be considered committed
   */
  return 0;
}
