#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
 
using  u64 = uint64_t;
using  u32 = uint32_t;
using  u16 = uint16_t;
 
 
u32 counter = 0; // I'm reasonably confident this is atomic
char hi[80]; 
// On GP2 (physical pin 4)
void gpio_callback(uint gpio, uint32_t events) 
{
    counter++;
}
 
bool repeating_timer_callback(struct repeating_timer *t) {
    u32 tmp = counter;
    counter = 0;
    
    sprintf(hi,"%d\n", tmp);
    Serial.print(hi);
    return true;
}
 
 
void setup() {

  
  // put your setup code here, to run once:
    Serial.begin(19200);
    delay(50);
    Serial.flush();
    delay(50);
    sprintf(hi,"Hertz\n");
    Serial.print(hi);
    gpio_set_irq_enabled_with_callback(5, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
 
    struct repeating_timer timer;
    add_repeating_timer_ms(-1000, repeating_timer_callback, NULL, &timer);

}

void loop() {
    int i=0;
}    
