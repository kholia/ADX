#include <Arduino.h>
#include "pdx_common.h"

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                             ATU CONTROL CONFIGURATION                                       *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*   
/*----------------------------------------------------------------------------------------------*
 * ATU control is an optional feature. When enabled thru the "#define ATUCTL 1" statement will  * 
 * create a short HIGH pulse (200 mSecs) thru the ATU designated pin.                           *
 * This signal can be used to reset an external ATU by means of a simple transitor interface    *
 * which either grounds or raises the RESET pin of the ATU being used. This pulse is created    *
 * whenever a band change occurs.                                                               *
 *----------------------------------------------------------------------------------------------*/
#ifdef ATUCTL

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*               ATU RESET FUNCTION SUPPORT                                                    *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*

uint16_t atu       =  ATU;
uint16_t atu_delay =  ATU_DELAY;
uint32_t tATU = 0;

//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*==*
//*                    ATU DEVICE MANAGEMENT                                                     *
//*this is an optional function that creates a brief pulse aimed to reset the ATU on band changes*
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*

/*---------------------------------------------
 * perform the pulse
 * note the HIGH will be cleared as part
 * of the loop() processing when the timer
 * exceeds tATU+ATU_DELAY which controls the
 * width of the pulse
 */
void flipATU() {

  setGPIO(atu, HIGH);
  setWord(&TSW, ATUCLK, true);
  tATU = millis();

#ifdef DEBUG
  _INFO;
#endif //DEBUG
}
/*---------------------------------------------------------------
 * check if the ATU pulse has expired and the pin needs to be 
 * turned LOW again
 */
void checkATU() {
  if ((millis() - tATU) > atu_delay && getWord(TSW, ATUCLK) == true) {
    setWord(&TSW, ATUCLK, false);
    setGPIO(atu, LOW);
  }
}  

/*-------------------------
 * Initialize port for ATU
 */
void initATU() {
  

  gpio_init(uint8_t(atu));
  gpio_set_dir (uint8_t(atu), GPIO_OUT);
  flipATU();

}

#endif //ATUCTL
