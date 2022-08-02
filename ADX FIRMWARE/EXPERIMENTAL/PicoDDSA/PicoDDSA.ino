
/*
 * PicoDDS 
 * Software DDS algorithm
 * Based on Project 17 pp 67-70 from Burkhard Kainka (DK7JD) in his book 
 * "RPi Pico Projects and Circuits"
 
 */
 
 #include "pico/stdlib.h"
 #include "hardware/pwm.h"
 #include "pico/multicore.h"

 uint32_t f_hi=0;

void setup() {
   set_sys_clock_khz(100000,true);
   Serial.begin(115200);
   while(true){
    if (Serial.available()){
      char ch=Serial.read();
      uint32_t f=Serial.parseInt();
      if (ch==70) f*=1000; //F
      if (ch==102) f*=1; //f
      Serial.println(f);
      ch=Serial.read();
      uint16_t pre=1;
      if (f<1908) pre =10;
      if (f<191) pre=100;
      if (f<20) pre=250;
      gpio_set_function(0,GPIO_FUNC_PWM);
      pwm_config cfg=pwm_get_default_config();
      pwm_config_set_clkdiv_int(&cfg,pre);
      pwm_init(0,&cfg,true);
      uint32_t wrap = 100000000 / pre / f;
      pwm_set_wrap(0,wrap-1);
      pwm_set_gpio_level(0,wrap/2);
      pwm_set_enabled(0,true);
    }
    }
}
void pwm_int() {
  pwm_clear_irq(2);
  f_hi++;
}

void loop(){}

void setup1(){
  uint32_t f=0;
  while (true) {
     pwm_config cfg=pwm_get_default_config();
     pwm_config_set_clkdiv_mode(&cfg,PWM_DIV_B_RISING);
     pwm_init(2,&cfg,false);
     gpio_set_function(5,GPIO_FUNC_PWM);
     pwm_set_irq_enabled(2,true);
     irq_set_exclusive_handler(PWM_IRQ_WRAP,pwm_int);
     irq_set_enabled(PWM_IRQ_WRAP,true);
     f_hi=0;
     uint32_t t=time_us_32()+2;
     while (t>time_us_32());
     pwm_set_enabled(2,true);
     t+=1000000;
     while (t>time_us_32());
     pwm_set_enabled(2,false);
     f=pwm_get_counter(2);
     f+=f_hi<<16;
     Serial.print(f);
     Serial.println(" Hz");
     
  }
}

void loop1() {}
