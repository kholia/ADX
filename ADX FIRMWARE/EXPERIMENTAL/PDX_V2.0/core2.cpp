#include <Arduino.h>
#include "pdx_common.h"




//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*                                      CORE2 Processing                                       *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
/*                                                                                              *
 * PDX will implement precise frequency measurement using the 2nd core the rp2040 has           *
 * these routines goes mostly for the calibration phase and the different frequency counting    *
 * algorithms used.                                                                             *
 * Code deployed on the 2nd core needs to be communicated with the primary core thru IPC        *
 * mechanisms for R/W common areas. Write in one core and read in the other is ok for non time  *
 * critical functions.                                                                          *
 * The Arduino IDE enables the dual core when detecting the setup1 & loop1 procedures, in this  *
 * case all functions are executed at setup1 where the actual flow is left looping so loop1 is  *
 * defined but no code on it will ever be executed.                                             *
 *----------------------------------------------------------------------------------------------*/

 //=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
//*                           PDX Calibration (automatic) and FSK counting algorithm                        *
//=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
/*------
   Înterrupt IRQ for edge counting overflow
  -----*/
void pwm_int() {
  pwm_clear_irq(pwm_slice);
  f_hi++;
}

#ifdef FSK_ADCZ
/*------------------------------------------------------------------------------------------*
   calibrateADC
   Calibrate the ADC zero level
*/
uint16_t calibrateADC(uint16_t min, uint16_t max) {
  return uint16_t((adc_max - adc_min) * 1.0 / 2.0) + adc_min;
}

/*-------------------------------------------------------------------------------------------*
   ADCreset
   restore all calibration values
*/
void ADCreset() {
  adc_min = ADCMAX;
  adc_max = ADCMIN;
  adc_zero = ADCZERO;
  adc_uh = adc_zero * 110 / 100;
  adc_ul = adc_zero * 90 / 100;
  ffmin = FSKMAX;
  ffmax = FSKMIN;
#ifdef DEBUG
  _TRACELIST("%s Timeout break QSTATE=0, recalibrate input level", __func__);
#endif //DEBUG
}

/*------------------------------------------------------------------------------------------*
   getADCsample
   collect an ADC sample running free.
   update the minimum and maximum
*/
uint16_t getADCsample() {
  uint16_t v = adc_read();
  if (v > adc_max) {
    adc_max = v;
    adc_zero = calibrateADC(adc_min, adc_max);
    adc_uh = adc_zero * 110 / 100;
    adc_ul = adc_zero * 90 / 100;
#ifdef DEBUG
    _TRACELIST("%s calibration (max) adc_max=%d adc_min=%d adc_Zero=%d\n", __func__, adc_max, adc_min, adc_zero);
#endif //DEBUG
    return v;
  }
  if (v <= adc_min) {
    adc_min = v;
    adc_zero = calibrateADC(adc_min, adc_max);
    adc_uh = adc_zero * 110 / 100;
    adc_ul = adc_zero * 90 / 100;
#ifdef DEBUG
    _TRACELIST("%s calibration (min) adc_max=%d adc_min=%d adc_Zero=%d\n", __func__, adc_max, adc_min, adc_zero);
#endif //DEBUG
    return v;
  }
  if (v >= adc_uh) {
    adc_high = true;
  }
  if (v <= adc_ul) {
    adc_low = true;
  }

  return  v;
}
#endif //FSK_ADCZ

/*=========================================================================================*
   CORE1
   2nd rp2040 core instantiated by defining setup1/proc1 procedures
   These procedures are used to run frequency measurement / time sensitive code
   the por1 procedure isn't never reached actually as the flow is left at an infinite loop
   at setup1
  =========================================================================================*/
void setup1() {
  /*-----------------------------------------------------------------*
     Core1   Setup procedure
     Enter processing on POR but restarted from core0 setup ()
    -----------------------------------------------------------------*/
  uint32_t t = 0;
  bool     b = false;
  /*--------------------------------------------*
     Wait for overall initialization to complete
    --------------------------------------------*/
  while (getWord(QSW, QWAIT) == false) {

#ifdef WDT
    wdt_reset();
#endif //WDT

    uint32_t t = time_us_32() + 2;
    while (t > time_us_32());
  }
  /*-------------------------------------------*
     Semaphore QWAIT has been cleared, proceed
     PWM counters operates as infinite loops
     therefore no loop1() is ever processed
    -------------------------------------------*/

  //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
  //* Automatic calibration procedure                                                                             *
  //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
  if (getWord(QSW, QCAL) == true) {
    delay(1000);
    calibrateLED();

    /*----
       Prepare Si5351 CLK2 for calibration process
      ---*/

    switch_RXTX(LOW);

    gpio_set_function(CAL, GPIO_FUNC_PWM); // GP9
    si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA); // Set for lower power for calibration
    si5351.set_clock_pwr(SI5351_CLK0, 0); // Enable the clock for calibration
    si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA); // Set for lower power for calibration
    si5351.set_clock_pwr(SI5351_CLK1, 0); // Enable the clock for calibration
    si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA); // Set for lower power for calibration
    si5351.set_clock_pwr(SI5351_CLK2, 1); // Enable the clock for calibration
    si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
    si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
    si5351.set_freq(Cal_freq * 100UL, SI5351_CLK2);

    /*--------------------------------------------*
       PWM counter used for automatic calibration
       -------------------------------------------*/
    fclk = 0;
    int16_t n = int16_t(CAL_COMMIT);
    cal_factor = 0;

    pwm_slice = pwm_gpio_to_slice_num(CAL);
    while (true) {
      /*-------------------------*
         setup PWM counter
        -------------------------*/
      pwm_config cfg = pwm_get_default_config();
      pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_RISING);
      pwm_init(pwm_slice, &cfg, false);
      gpio_set_function(CAL, GPIO_FUNC_PWM);

      pwm_set_irq_enabled(pwm_slice, true);
      irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_int);
      irq_set_enabled(PWM_IRQ_WRAP, true);
      f_hi = 0;

      /*---------------------------*
         PWM counted during 1 sec
        ---------------------------*/
      t = time_us_32() + 2;
      while (t > time_us_32());
      pwm_set_enabled(pwm_slice, true);
      t += 1000000;
      while (t > time_us_32());
      pwm_set_enabled(pwm_slice, false);

      /*----------------------------*
         recover frequency in Hz
        ----------------------------*/
      fclk = pwm_get_counter(pwm_slice);
      fclk += f_hi << 16;
      error = fclk - Cal_freq;

      if (labs(error) > int32_t(CAL_ERROR)) {
        b = !b;
        if (b) {
          setLED(TX, false);
        } else {
          rstLED(TX, false);
        }
        if (error < 0) {
          cal_factor = cal_factor - CAL_STEP;
        } else {
          cal_factor = cal_factor + CAL_STEP;
        }
        si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
      } else {
        n--;
        if (n == 0) {


#ifdef EE
          updateEEPROM();
#endif //EE

          while (true) {
#ifdef WDT
            wdt_reset();
#endif //WDT
#ifdef EE
            checkEEPROM();
#endif //EE
          }
          while (true) {
            resetLED();
            setLED(JS8, true);
            setLED(FT4, false);
            delay(1000);
          }
        }
      }
    }
  } //Auto calibration mode

  //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
  //* FSK detection algorithm                                                                                     *
  //* Automatic input detection algorithm                                                                         *
  //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
  if (getWord(QSW, QFSK) == true) {
    ffsk = 0;

#ifdef FSK_ZCD
    /*----------------------------------------*
       ZCD algorithm
       defined by FSK_ZCD
       this is based on a pseudo cross detect
       where the rising edge is taken as a
       false cross detection followed by next
       edge which is also a false zcd but
       at the same level thus measuring the
       time between both will yield a period
       measurement proportional to the real
       period of the signal as measured
       two sucessive rising edges
       Measurements are made every 1 mSec
      ---------------------------------------*/
    uint16_t cnt = 100;
    pwm_slice = pwm_gpio_to_slice_num(FSK);

    /*--------------------------------------------------------------*
       main counting algorithm cycle
      --------------------------------------------------------------*/
    while (true) {
      pwm_config cfg = pwm_get_default_config();
      pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_RISING);
      pwm_init(pwm_slice, &cfg, false);
      gpio_set_function(FSK, GPIO_FUNC_PWM);
      pwm_set_irq_enabled(pwm_slice, true);
      irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_int);
      irq_set_enabled(PWM_IRQ_WRAP, true);
      f_hi = 0;
#ifdef FSK_ZCD
      uint32_t t = time_us_32() + 2;                     //Allow all the settings to stabilize
      while (t > time_us_32());                          //
      uint16_t j = FSK_RA;                               //
      uint32_t dt = 0;                                    //
      while (j > 0) {                                    //Establish a running average over <j> counts
        uint32_t pwm_cnt = pwm_get_counter(pwm_slice);  //Get current pwm count
        pwm_set_enabled(pwm_slice, true);               //enable pwm count
        while (pwm_get_counter(pwm_slice) == pwm_cnt) {} //Wait till the count change
        pwm_cnt = pwm_get_counter(pwm_slice);           //Measure that value
        uint32_t t1 = time_us_32();                     //Mark first tick (t1)
        while (pwm_get_counter(pwm_slice) == pwm_cnt) {} //Wait till the count change (a rising edge)
        uint32_t t2 = time_us_32();                     //Mark the second tick (t2)
        pwm_set_enabled(pwm_slice, false);              //Disable counting
        dt = dt + (t2 - t1);                            //Add to the RA total
        j--;                                            //Loop
      }                                                  //

      if (dt != 0) {                                     //Prevent noise to trigger a nul measurement
        double dx = 1.0 * dt / double(FSK_RA);          //
        double f = double(FSK_USEC) / dx;               //Ticks are expressed in uSecs so convert to Hz
        double f1 = round(f);                           //Round to the nearest integer
        ffsk = uint32_t(f1);                            //Convert to long integer for actual usage
        if (ffsk >= FSKMIN && ffsk <= FSKMAX) {         //Only yield a value if within the baseband
          fsequences[nfsi] = f;
          rp2040.fifo.push_nb(nfsi);                  //Use the rp2040 FIFO IPC to communicate the new frequency
          nfsi = (nfsi + 1) % NFS;
        }                                               //
      }                                                  //
      t = time_us_32() + FSK_SAMPLE;                     //Now wait for 1 mSec till next sample
      while (t > time_us_32()) ;
#endif //FSK_ZCD
    }  //end FSK (ZCD or PEG) loop
#endif //FSK_ZCD

#ifdef FSK_ADCZ
    /*----------------------------------------*
       ADCZalgorithm
       defined by FSK_ADCZ
       This algorithm samples the ADC port
       at full speed (500 KS/sec) and identify
       zero crossings from + to - values
       Two consecutive epoch are taken and
       the frequency computed from them.
       Actual values obtained from the ADC
       would change based on the VOL setting
       and therefore an adaptive level cal is
       performed. A squelch zone is defined to
       avoid readings with insuficient level
       A direct sampling would take several
       false signals which would lead to false
       freq readings, so an extensive adaptive
       filter is implemented as a finite state
       machine (FSM) in order to ensure the
       consistency of the two epoch taken
      ----------------------------------------*/

    /*------------------------------------*
       ADC initialization and setup
    */
    adc_init();
    adc_gpio_init( ADC_PIN);
    adc_select_input( ADC_NUM);

    /*----------------------------------------------------*
       Signal processing
       This processing is heavy filtered of inconsistent
       states produced by noise in the signal
       Filtering is performed using a finite state machine
       Wrong intermediate states leading to freq reading
       errors are assumed to be produced randomly and thus
       associated to white noise. The appareance of them
       can be assumed as a hidden system state which needs
       to be filtered. The FSM implemented knows at all
       samples which will the right combination and thus
       is able to predict what is the valid next sample,
       if a different state is observed then it can be
       regarded as noise or corruption of the sample
       stream. Since the algorithm is used to detect
       weak signals which change slowly there is not a
       critical need for samples within the window allowed
       and thus it's safer to drop a corrupt freq reading
       than to keep it and ruin the output stream, then
       the FSM is reset in any detected corrupt state and
       values recalibrated. This corruption can be caused
       by noise but also because the input stream ceased
       and the ADC readings are just the "sound of the
       silence", in this case the detection become silent
       and stop updating the frequency readings to the
       upcall procedure. This behaviour can be associated
       with some liberties to a Kalman filter construct
      ----------------------------------------------------*/

    while (true) {

      switch (QSTATE) {
        //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
        //* State 0 - Wait for the signal to be positive to start a counting cycle       *
        //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
        case 0 : {
            adc_high = false;
            adc_low = false;

            adc_v1 = getADCsample();             //Wait till two sucessive readings are positive, exit if none in 1 mSec
            adc_t1 = time_us_32();
            uint32_t tstop = adc_t1 + 1000;
            while (true) {
              adc_v2 = getADCsample();
              adc_t2 = time_us_32();
              if (adc_v1 >= adc_zero && adc_v2 >= adc_zero) {
                QSTATE = 1;
                break;
              }
              adc_v1 = adc_v2;
              adc_t1 = adc_t2;
              if (time_us_32() > tstop) {
                QSTATE = 0;
                ADCreset();
                break;
              }

            }
          } //Q(0)
        //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
        //* State 1 - Wait for a zero crossing to get the first epoch                    *
        //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
        case 1 : {
            adc_v1 = getADCsample();             //Last state was at least one pair of positive values, so wait for a crossing
            adc_t1 = time_us_32();               //accept but ignore sucessive pairs of positive values, the state looks
            uint32_t tstop = adc_t1 + 1000;      //for one positive and the next negative. Any other combination reset the finite state machine

            while (true) {
              adc_v2 = getADCsample();
              adc_t2 = time_us_32();
              if (adc_v1 >= adc_zero && adc_v2 <= adc_zero) {
                t1[0] = adc_t1;
                t2[0] = adc_t2;
                v1[0] = adc_v1;
                v2[0] = adc_v2;
                QSTATE = 2;
                break;
              }

              if (adc_v1 >= adc_zero && adc_v2 >= adc_zero) {
                adc_v1 = adc_v2;
                adc_t1 = adc_t2;
                if (time_us_32() > tstop) {
                  QSTATE = 0;
                  break;
                }
                continue;
              }
              QSTATE = 0;
              break;


            }

          }  //Q(1)
        //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
        //* State 2 - Wait for the signal to become fully negative                       *
        //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
        case 2 : {                                                //A cross between positive and negative was detected, if another crossing is detected
            adc_v1 = getADCsample();                       //then sampling was fast enough to capture another sucessive crossing, thus the mark
            adc_t1 = time_us_32();                         //is updated. If two sucessive negative values are detected the crossing is completed
            uint32_t tstop = adc_t1 + 1000;                //and the FSM is advanced to next state. Any other combination is weird and reset the FSM

            while (true) {
              adc_v2 = getADCsample();
              adc_t2 = time_us_32();
              if (adc_v1 <= adc_zero && adc_v2 <= adc_zero) {
                QSTATE = 3;
                break;
              }

              if (adc_v1 >= adc_zero && adc_v2 <= adc_zero) {
                t1[0] = adc_t1;
                t2[0] = adc_t2;
                v1[0] = adc_v1;
                v2[0] = adc_v2;

                adc_v1 = adc_v2;
                adc_t1 = adc_t2;
                if (time_us_32() > tstop) {
                  QSTATE = 0;
                  break;
                }
                continue;
              }
              QSTATE = 0;
              break;
            }
          } //Q(2)
        //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
        //* State 3 - Wait for the signal to become fully positive, start 2nd checkpoint *
        //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
        case 3: {                                                  //At this point at least two successive negative values has been obtained
            adc_v1 = getADCsample();                        //the sample stream is now evaluated and any value other than two sucessive positive values
            adc_t1 = time_us_32();                          //is ignored. So when the signal swing back to positive the FSM is advanced.
            uint32_t tstop = adc_t1 + 1000;

            while (true) {
              adc_v2 = adc_read();
              adc_t2 = time_us_32();
              if (adc_v1 >= adc_zero && adc_v2 >= adc_zero) {
                QSTATE = 4;
                break;
              }
              adc_v1 = adc_v2;
              adc_t1 = adc_t2;
              if (time_us_32() > tstop) {
                QSTATE = 0;
                break;
              }

            }
          } //Q(3)
        //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
        //* State 4 - Wait for the 2nd zero crossing and take the epoch when detected    *
        //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
        case 4: {                                                  //At this point at least two sucessive positive values has been detected. Others might follow
            adc_v1 = getADCsample();                        //which are ignored until another crossing is detected. This is similar to state <1> but for
            adc_t1 = time_us_32();                          //the next cycle. Samples other than both positive or a crossing reset the FSM
            uint32_t tstop = adc_t1 + 1000;

            while (true) {
              adc_v2 = getADCsample();
              adc_t2 = time_us_32();
              if (adc_v1 >= adc_zero && adc_v2 <= adc_zero) {
                t1[1] = adc_t1;
                t2[1] = adc_t2;
                v1[1] = adc_v1;
                v2[1] = adc_v2;
                QSTATE = 5;
                break;
              }
              if (adc_v1 >= adc_zero && adc_v2 >=  adc_zero) {
                adc_v1 = adc_v2;
                adc_t1 = adc_t2;
                if (time_us_32() > tstop) {
                  QSTATE = 0;
                  break;
                }
                continue;
              }
              QSTATE = 0;
              break;
            }
          } //Q(4)

        //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
        //* State 5 - Wait for the signal to stabilize at negateive values               *
        //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
        case 5: {                                                     //At this point a crossing has been detected, the sample stream is inspected looking for another
            adc_v1 = getADCsample();                           //crossing and values are updated. When two sucessive samples are negative the FSM is advanced
            adc_t1 = time_us_32();                             //to status 6.
            uint32_t tstop = adc_t1 + 1000;

            while (true) {
              adc_v2 = getADCsample();
              adc_t2 = time_us_32();
              if (adc_v1 <= adc_zero && adc_v2 <= adc_zero) {
                QSTATE = 6;
                break;
              }

              if (adc_v1 >= adc_zero && adc_v2 <= adc_zero) {
                t1[0] = adc_t1;
                t2[0] = adc_t2;
                v1[0] = adc_v1;
                v2[0] = adc_v2;

                adc_v1 = adc_v2;
                adc_t1 = adc_t2;
                if (time_us_32() > tstop) {
                  QSTATE = 0;
                  break;
                }
                continue;
              }
              QSTATE = 0;
              break;
            }
          } //Q(5)
        //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
        //* State 6 - Two epoch available, compute the frequency                         *
        //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
        case 6: {                                                                    //Datum for two sucessive crossings has been collected at this point
            //then a more precise computation of the exact epoch of each crossing
            double m = ((v2[0] - v1[0]) * 1.0 / (t2[0] - t1[0]) * 1.0);          //is performed. The frequency is computed as the projection of the difference
            double t0s = t1[0] + uint32_t((adc_zero - v1[0]) * 1.0 / m);         //between two sucessive crossings projected to a full second.
            m = ((v2[1] - v1[1]) * 1.0 / (t2[1] - t1[1]) * 1.0);
            double t1s = t1[1] + uint32_t((adc_zero - v1[1]) * 1.0 / m);
            /*---------------------------------------------------------*
               This operates as a squelch by insuring that during the
               frequency measurement both positive and negative values
               went in excess of the calibrated zero level +/-10%
               and not computing the frequency if the signal was too
               low because it would create large errors if so
              ---------------------------------------------------------*/
            //if (adc_low == false || adc_high == false) {
            if (adc_high == false) {
              QSTATE = 7;
              break;
            }

            /*----------------------------------------------------------*
               If a valid epoch pair is detected then the frequency is
               computed, which is validated by allowing only baseband
               valid values to pass, this removes any high pitch reading
               because of measuring noise or a weak signal.
               Frequency is sent to the upcall caller working at core0
               and a rounding mechanism is applied (Dhiru Kholia's fix)
              ----------------------------------------------------------*/
            if ((t1s > t0s)) {
              double f = 1000000 / (t1s - t0s);
              if (f >= double(FSKMIN) && f <= double(FSKMAX)) {
                if (f < ffmin) {
                  ffmin = f;
                }
                if (f > ffmax) {
                  ffmax = f;
                }
                /*-----------------------------*
                   this is a valid f epoch
                  -----------------------------*/
                /*
                  fsequences[nfsi] = f;
                  rp2040.fifo.push_nb(nfsi);                  //Use the rp2040 FIFO IPC to communicate the new frequency
                  nfsi = (nfsi + 1) % NFS;
                  #ifdef DEBUG
                   _TRACELIST("%s dt=%ld dx=%.3f f=%.3f f1=%.3f ffsk=%ld\n",__func__,dt,dx,f,f1,ffsk);
                  #endif //DEBUG
                */
                ffsk = uint32_t(round(f));
                rp2040.fifo.push_nb(ffsk);                  //Use the rp2040 FIFO IPC to communicate the new frequency
              }                                                   //
            }
            QSTATE = 7;

          } //Q(6)
        //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
        //* State 7 - Wait for the next measurement                                      *
        //*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
        case 7: {                                                                     //In this state the FSM rest for a given delay
            sleep_ms(ADCSAMPLE);
            QSTATE = 0;
          }  //Q(7)

      }  //FSM(QSTATE) logic

    } //FSM(QSTATE) infinite loop
#endif //FSK_ADCZ
  }
}
/*==========================================================================================================*/
