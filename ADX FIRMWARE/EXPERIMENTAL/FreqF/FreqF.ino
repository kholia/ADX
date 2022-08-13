  
/*------------------------
 * FreqF
 * Frequency counting algorithm
 * Based on research performed for the PDX project (ZCD algorithm) 
 */

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "pico/multicore.h"

#define FSK  27
#define FSKMIN             300    //Minimum FSK frequency computed
#define FSKMAX            2500    //Maximum FSK frequency computed

#define FSK_USEC                  1000000
#define FSK_SAMPLE                  10000
#define FSK_IDLE      5*FSK_SAMPLE*FSK_RA
#define FSK_ERROR                       4
#define FSK_RA                         20

uint32_t f_hi;
int      pwm_slice;  
uint32_t ffsk     = 0;

void setup() {

  //set_sys_clock_khz(200000,true);
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("FreqF Frequency counter ready");
   
}

void loop() {}
void pwm_int() {
      pwm_clear_irq(pwm_slice);
      f_hi++;
}
/*=========================================================================================*
 * CORE1                                                                                   *
 * 2nd rp2040 core instantiated by defining setup1/proc1 procedures                        *
 * These procedures are used to run frequency measurement / time sensitive code            *
 * the por1 procedure isn't never reached actually as the flow is left at an infinite loop *
 * at setup1                                                                               *
 *=========================================================================================*/
void setup1() {

/*-----------------------------------------------------------------*
 * Core1   Setup procedure                                         *
 * Enter processing on POR but restarted from core0 setup ()       *
 *-----------------------------------------------------------------*/
uint32_t t = 0;
bool     b = false;
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//* FSK detection algorithm                                                                                     *
//* Automatic input detection algorithm                                                                         *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*   

      ffsk=0;
      uint16_t cnt=100;
      pwm_slice=pwm_gpio_to_slice_num(FSK);

/*--------------------------------------------------------------*      
 * main counting algorithm cycle                                *
 *--------------------------------------------------------------*/
      while (true) {
          pwm_config cfg=pwm_get_default_config();
          pwm_config_set_clkdiv_mode(&cfg,PWM_DIV_B_RISING);
          pwm_init(pwm_slice,&cfg,false);
          gpio_set_function(FSK,GPIO_FUNC_PWM);
          pwm_set_irq_enabled(pwm_slice,true);
          irq_set_exclusive_handler(PWM_IRQ_WRAP,pwm_int);
          irq_set_enabled(PWM_IRQ_WRAP,true);
          f_hi=0;
         /*----------------------------------------*
           * ZCD algorithm                         *
           * defined by FSK_ZCD                    *
           * this is based on a pseudo cross detect*
           * where the rising edge is taken as a   *
           * false cross detection followed by next*
           * edge which is also a false zcd but    *
           * at the same level thus measuring the  *
           * time between both will yield a period *
           * measurement proportional to the real  *
           * period of the signal as measured      *
           * two sucessive rising edges            *
           * Measurements are made every 1 mSec    *
           *---------------------------------------*/             
             uint32_t t=time_us_32()+2;                         //Allow all the settings to stabilize
             while (t>time_us_32());                            //
             uint16_t j=FSK_RA;                                 //
             uint32_t dt=0;                                      //
             while (j>0) {                                      //Establish a running average over <j> counts
                uint32_t pwm_cnt=pwm_get_counter(pwm_slice);    //Get current pwm count
                pwm_set_enabled(pwm_slice,true);                //enable pwm count
                while (pwm_get_counter(pwm_slice) == pwm_cnt){} //Wait till the count change
                pwm_cnt=pwm_get_counter(pwm_slice);             //Measure that value
                uint32_t t1=time_us_32();                       //Mark first tick (t1)
                while (pwm_get_counter(pwm_slice) == pwm_cnt){} //Wait till the count change (a rising edge)
                uint32_t t2=time_us_32();                       //Mark the second tick (t2)
                pwm_set_enabled(pwm_slice,false);               //Disable counting
                dt=dt+(t2-t1);                                  //Add to the RA total
                j--;                                            //Loop
             }                                                  //
             if (dt != 0) {                                     //Prevent noise to trigger a nul measurement
                double dx=1.0*dt/double(FSK_RA);                //
                double f=double(FSK_USEC)/dx;                   //Ticks are expressed in uSecs so convert to Hz
                double f1=round(f);                             //Round to the nearest integer 
                ffsk=uint32_t(f1);                              //Convert to long integer for actual usage
                if (ffsk >= FSKMIN && ffsk <= FSKMAX) {         //Only yield a value if within the baseband 
                   Serial.print(ffsk);
                   Serial.println(" Hz");
                }                                               //
             }                                                  //
             t=time_us_32()+FSK_SAMPLE;                         //Now wait for 1 mSec till next sample
             while (t>time_us_32()) ;
            
        }  //end FSK loop  
}

void loop1() {}       
 
