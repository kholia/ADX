
/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#include <Arduino.h>
#include <stdint.h>
#include "Wire.h"
#include <EEPROM.h>

#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "hardware/sync.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"
#include "hardware/watchdog.h"


// This example drives a PWM output at a range of duty cycles, and uses
// another PWM slice in input mode to measure the duty cycle. You'll need to
// connect these two pins with a jumper wire:
const uint OUTPUT_PIN = 2;
const uint MEASURE_PIN = 5;
char hi[80];

int led = LED_BUILTIN; // the PWM pin the LED is attached to
int brightness = 0;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED


// Frequency measured in kHz
float measure_frequency(uint gpio) {
    // Only the PWM B pins can be used as inputs.
    assert(pwm_gpio_to_channel(gpio) == PWM_CHAN_B);
    uint slice_num = pwm_gpio_to_slice_num(gpio);

    // Count once for every 100 cycles the PWM B input is high
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_RISING);
    pwm_config_set_clkdiv(&cfg, 1.f); //set by default, increment count for each rising edge
    pwm_init(slice_num, &cfg, false);  //false means don't start pwm
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    
    pwm_set_enabled(slice_num, true);
    sleep_ms(10);
    pwm_set_enabled(slice_num, false);
    
    uint16_t counter = (uint16_t) pwm_get_counter(slice_num);
    float freq =   counter / 4.0; // / 10.f;
    return freq;

}




float measure_duty_cycle(uint gpio) {
    // Only the PWM B pins can be used as inputs.
    assert(pwm_gpio_to_channel(gpio) == PWM_CHAN_B);
    uint slice_num = pwm_gpio_to_slice_num(gpio);

    // Count once for every 100 cycles the PWM B input is high
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_HIGH);
    pwm_config_set_clkdiv(&cfg, 100);
    pwm_init(slice_num, &cfg, false);
    gpio_set_function(gpio, GPIO_FUNC_PWM);

    pwm_set_enabled(slice_num, true);
    sleep_ms(10);
    pwm_set_enabled(slice_num, false);
    float counting_rate = clock_get_hz(clk_sys) / 100;
    float max_possible_count = counting_rate * 0.01;
    return pwm_get_counter(slice_num) / max_possible_count;
}

const float test_duty_cycles[] = {
        0.f,
        0.1f,
        0.5f,
        0.9f,
        1.f
};

/*
    // Configure PWM slice and set it running
    const uint count_top = 1000;
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_wrap(&cfg, count_top);
    pwm_init(pwm_gpio_to_slice_num(OUTPUT_PIN), &cfg, true);

    // Note we aren't touching the other pin yet -- PWM pins are outputs by
    // default, but change to inputs once the divider mode is changed from
    // free-running. It's not wise to connect two outputs directly together!
    gpio_set_function(OUTPUT_PIN, GPIO_FUNC_PWM);
*/


void setup() {
  // put your setup code here, to run once:

    pinMode(led, OUTPUT);

    Serial.begin(19200);
    while (!Serial) {
      
    }
    Serial.flush();
    delay(50);
    sprintf(hi,"\nFrequency measurement algorithm\n");
    Serial.print(hi);


}

void loop() {
  // put your main code here, to run repeatedly:

    // For each of our test duty cycles, drive the output pin at that level,
    // and read back the actual output duty cycle using the other pin. The two
    // values should be very close!
    float f = measure_frequency(MEASURE_PIN);
    sprintf(hi,"Frequency = %8.2f%\n",f*100.f);
    Serial.print(hi);
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
