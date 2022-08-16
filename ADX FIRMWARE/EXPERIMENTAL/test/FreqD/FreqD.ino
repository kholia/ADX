/*-------------------------------------------------------------------------------------*
 * FreqC
 * Test frequency counter algorithm for PDX project
 * This algorithm uses the medium resolution Timer timestamp (uSecs resolution)
 *-------------------------------------------------------------------------------------*/
#include <Arduino.h>
#include <stdint.h>
#include <si5351.h>
#include "Wire.h"
#include <EEPROM.h>

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "hardware/sync.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"
#include "hardware/watchdog.h"
 
int led = LED_BUILTIN; // the PWM pin the LED is attached to
int brightness = 0;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED
uint32_t counter = 0; // I'm reasonably confident this is atomic
char hi[80]; 
static repeating_timer_t timers[10] = {0,};
int timer = 1;

/*-----
 * timestamp
 */
uint64_t t1;
uint64_t t2;

/*-----
 * Process average
 */
static uint64_t sum=0;
static uint64_t n=0;

uint64_t fMIN=1000000;
uint64_t fMAX=0;

int i=0;


#define SAMPLE -50        //mSecs negative means constant sampling
bool success=false;

/*------
 * GPIO05 IRQ on edge
 */
void gpio_callback(uint gpio, uint32_t events) 
{
    t2=time_us_64();
    sum=sum+t2-t1;
    t1=time_us_64();
    n++;
}

/*----- 
 * This is called periodically to collect results
 */
bool repeating_timer_callback(struct repeating_timer *t) {
    int64_t f=1000000;
    
    if (sum !=0 && n!=0) {
       f=(n*f)/sum;
    } else {
       f=0.0;
    }
    
// "%" PRId64 "\n"
    i++;
    if (i>=20) {
       sprintf(hi,"n=%ld sum=%ld f=%ld\n", uint32_t(n),uint32_t(sum),uint32_t(f));    
       Serial.print(hi);
       i=0;
    }   
    sum=0;
    n=0;
    
    return true;
}
/*----- 
 * Setup execution
 */
void setup() {

//    Initial value for timestamp

    t1=time_us_64();
    t2=time_us_64();


//    establish IRQ for GPIO5

    gpio_set_irq_enabled_with_callback(5, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
 
//   struct repeating_timer timer;   

    success = add_repeating_timer_ms( -10, repeating_timer_callback, NULL, &timers[timer] );

//  Just a "I'm alive" beacon 
    
    pinMode(led, OUTPUT);

// Serial port to collect results

    Serial.begin(19200,SERIAL_8N2);
    while (!Serial) {
    }
    delay(2);
    Serial.flush();
    Serial.setTimeout(50);    
    
    Serial.println("FreqD here");
    
}

void loop() {

/*-----
 * Really do nothing as everything is performed driven by interruptions
 * but periodically glow up and down the led to show aliveness
 */
  // set the brightness
  analogWrite(led, brightness);

  // change the brightness for next time through the loop:
  brightness = brightness + fadeAmount;

  // reverse the direction of the fading at the ends of the fade:
  if (brightness <= 0 || brightness >= 255) {
    fadeAmount = -fadeAmount;
  }

  // wait for 30 milliseconds to see the dimming effect
  delay(30);

}    
