/*-------------------------------------------------------------------------------------*
 * FreqB
 * Test frequency counter algorithm for PDX project
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

#define MILLIS 1000
bool success=false;



// On GP2 (physical pin 4)
void gpio_callback(uint gpio, uint32_t events) 
{
    counter++;
}
 
bool repeating_timer_callback(struct repeating_timer *t) {
    uint32_t tmp = counter;
    counter = 0;
    
    sprintf(hi,"f=%ld\n", tmp);
    Serial.print(hi);
    return true;
}
 
void setup() {

//    associate gpio 5 IRQ

    gpio_set_irq_enabled_with_callback(5, GPIO_IRQ_LEVEL_LOW, true, &gpio_callback);
 
//   struct repeating_timer timer;   

    success = add_repeating_timer_ms( MILLIS, repeating_timer_callback, NULL, &timers[timer] );
    
    pinMode(led, OUTPUT);
    Serial.begin(19200,SERIAL_8N2);
    while (!Serial) {
    }
    delay(2);
    Serial.flush();
    Serial.setTimeout(50);    
    
    Serial.println("FreqB here");
    

}

void loop() {
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
