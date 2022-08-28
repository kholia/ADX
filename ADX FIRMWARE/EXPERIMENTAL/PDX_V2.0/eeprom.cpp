#include <Arduino.h>
#include "pdx_common.h"

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                             EEPROM CONTROL CONFIGURATION                                    *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*   
/*----------------------------------------------------------------------------------------------*
 * Persistence is managed by means of storing critical configuration values on EEPROM           *
 * Since the number of read/write cycles of the EEPROM are finite (and not that big) care is    *
 * taken of reducing the cycles by committing to EEPROM only when value changes get stable      *
 * the rp2040 processor doesn't have a true EEPROM, it's emulated on flash memory instead       *
 * flash memory tolerates even less cycles. 
 */
#ifdef EE
uint32_t tout = 0;
uint16_t eeprom_tout = EEPROM_TOUT;
/*------------------------------------------------------------------------------*
   updateEEPROM
   selectively sets values into EEPROM
  ------------------------------------------------------------------------------*/
void updateEEPROM() {
  uint16_t save = EEPROM_SAVED;
  uint16_t build = BUILD;

  EEPROM.put(EEPROM_TEMP, save);
  EEPROM.put(EEPROM_BUILD, build);
  EEPROM.put(EEPROM_CAL, cal_factor);
  EEPROM.put(EEPROM_MODE, mode);
  EEPROM.put(EEPROM_BAND, Band_slot);

#ifdef TERMINAL
#ifdef ATUCTL
  EEPROM.put(EEPROM_ATU, atu);
  EEPROM.put(EEPROM_ATU_DELAY, atu_delay);
#endif //ATUCTL

  EEPROM.put(EEPROM_BOUNCE_TIME, bounce_time);
  EEPROM.put(EEPROM_SHORT_TIME, short_time);
  EEPROM.put(EEPROM_MAX_BLINK, max_blink);
  EEPROM.put(EEPROM_EEPROM_TOUT, eeprom_tout);

#endif //TERMINAL

  EEPROM.commit();

#ifdef DEBUG
  _INFOLIST("%s commit()\n", __func__);
#endif //DEBUG

  setWord(&SSW, SAVEEE, false);

#ifdef DEBUG
  _INFOLIST("%s save(%d) cal(%d) m(%d) slot(%d) save=%d build=%d\n", __func__, save, cal_factor, mode, Band_slot, save, build);
#endif //DEBUG

}

/*------------------------------------------------------------------------------*
   resetEEPROM
   reset to pinche defaults
  ------------------------------------------------------------------------------*/
void resetEEPROM() {
  uint16_t save = EEPROM_SAVED;
  uint16_t build = BUILD;

  mode = 0;
  Band_slot = 0;
  //* Retain calibration cal_factor=0;

#ifdef TERMINAL

#ifdef ATUCTL
  atu        = ATU;
  atu_delay  = ATU_DELAY;
#endif //ATUCTL

  bounce_time = BOUNCE_TIME;
  short_time = SHORT_TIME;
  max_blink  = MAX_BLINK;
  eeprom_tout = EEPROM_TOUT;

#endif //TERMINAL

  updateEEPROM();
}

/*------
   checkEEPROM
   check if there is a pending EEPROM save that needs to be committed
*/
void checkEEPROM() {
  if ((millis() - tout) > eeprom_tout && getWord(SSW, SAVEEE) == true ) {
#ifdef DEBUG
    _INFOLIST("%s() Saving EEPROM...\n", __func__);
#endif //DEBUG

    updateEEPROM();
  }
}
/*----------------------------------------
 * flag EEPROM to update after a timeout
 */
void flagEEPROM() {
    tout = millis();
    setWord(&SSW, SAVEEE, true);

}
/*----------------------------------------
 * initEEPROM
 * initialize values from EEPROM on start
 * handle the case of an empty EEPROM or 
 * a reset of the values stored on it
 */
void initEEPROM() {



  EEPROM.begin(512);

#ifdef DEBUG
  _INFOLIST("%s: EEPROM reserved (%d)\n", __func__, EEPROM.length());
#endif //DEBUG
  
  uint16_t temp = 0;
  uint16_t save = EEPROM_SAVED;
  uint16_t build = 0;


  EEPROM.get(EEPROM_TEMP, temp);
  EEPROM.get(EEPROM_BUILD, build);

#ifdef DEBUG
  _INFOLIST("%s EEPROM retrieved temp(%d) & Build(%d) BUILD=%d\n", __func__, temp, build, uint16_t(BUILD));
#endif //DEBUG

#ifdef EEPROM_CLR
  temp = -1;
#endif //EEPROM_CLR


  if (build != uint16_t(BUILD)) {
    resetEEPROM();
#ifdef DEBUG
    _INFOLIST("%s EEPROM Reset Build<> cal(%ld) m(%d) slot(%d)\n", __func__, cal_factor, mode, Band_slot);
#endif //DEBUG

  }

  if (temp != save) {

    updateEEPROM();

#ifdef DEBUG
    _INFOLIST("%s EEPROM Reset cal(%ld) m(%d) slot(%d)\n", __func__, cal_factor, mode, Band_slot);
#endif //DEBUG

  } else {

    /*-----------------------------------------------*
       get configuration initialization from EEPROM  *
      ------------------------------------------------*/

    EEPROM.get(EEPROM_CAL, cal_factor);

    /*---- Kludge Fix
           to overcome wrong initial values, should not difficult calibration
    */
    if (cal_factor < -31000) {
      cal_factor = 0;
    }
    /* end of kludge */

    EEPROM.get(EEPROM_MODE, mode);
    EEPROM.get(EEPROM_BAND, Band_slot);

#ifdef TERMINAL

#ifdef ATUCTL
    EEPROM.get(EEPROM_ATU, atu);
    EEPROM.get(EEPROM_ATU_DELAY, atu_delay);
#endif //ATUCTL

    EEPROM.get(EEPROM_BOUNCE_TIME, bounce_time);
    EEPROM.get(EEPROM_SHORT_TIME, short_time);
    EEPROM.get(EEPROM_MAX_BLINK, max_blink);
    EEPROM.get(EEPROM_EEPROM_TOUT, eeprom_tout);

#endif //TERMINAL


#ifdef DEBUG
    _INFOLIST("%s EEPROM Read cal(%ld) m(%d) slot(%d)\n", __func__, cal_factor, mode, Band_slot);
#endif //DEBUG
  }
}
#endif //EE
