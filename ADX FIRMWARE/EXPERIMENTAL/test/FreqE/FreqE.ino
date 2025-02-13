  
/*------------------------
 * FreqE
 * Frequency counting algorithm
 * Based on Project 17 pp 67-70 from Burkhard Kainka (DK7JD) in his book 
 * "RPi Pico Projects and Circuits"
 */

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "pico/multicore.h"

//#define GPIO_FSK        5
#define GPIO_FSK  27

uint32_t f_hi;
int      pwm_slice;  

void setup() {

  //set_sys_clock_khz(200000,true);
  Serial.begin(115200);
  
 
}

void loop() {}

   void pwm_int() {
      pwm_clear_irq(pwm_slice);
      f_hi++;
   }
   void setup1(){  
      uint32_t f=0;
      gpio_set_function(0,GPIO_FUNC_PWM);
      pwm_set_wrap(0,1); //PWM 62,5 MHz
      pwm_set_gpio_level(0,1);
      pwm_set_enabled(0,true);
      pwm_slice=pwm_gpio_to_slice_num(GPIO_FSK);
      
      while (true) {
          pwm_config cfg=pwm_get_default_config();
          pwm_config_set_clkdiv_mode(&cfg,PWM_DIV_B_RISING);
          pwm_init(pwm_slice,&cfg,false);
          gpio_set_function(GPIO_FSK,GPIO_FUNC_PWM);
          pwm_set_irq_enabled(pwm_slice,true);
          irq_set_exclusive_handler(PWM_IRQ_WRAP,pwm_int);
          irq_set_enabled(PWM_IRQ_WRAP,true);
          f_hi=0;
          uint32_t t=time_us_32()+2;
          while (t>time_us_32());
          pwm_set_enabled(pwm_slice,true);
          t+=1000000;
          while (t>time_us_32());
           pwm_set_enabled(pwm_slice,false);
           f=pwm_get_counter(pwm_slice);
           f+=f_hi<<16;
           Serial.print(f);
           Serial.println(" Hz");
          }
     }
    void loop1() {}       
 
