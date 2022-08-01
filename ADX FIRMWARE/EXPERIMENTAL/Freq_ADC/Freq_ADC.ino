/**
 * Copyright (c) 2021 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * FreqADC is a test contuit to evaluate a frequency counter algorithm based on zero crossing
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
#include "pico/binary_info.h"

#include <Arduino.h>
#include <stdint.h>
#include "Wire.h"
#include <EEPROM.h>

#include "hardware/sync.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"
#include "hardware/watchdog.h"
 
int led = LED_BUILTIN; // the PWM pin the LED is attached to
int brightness = 0;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED
char hi[80]; 

/* Example code to extract analog values from a microphone using the ADC
   Connections on Raspberry Pi Pico board, other boards may vary.
   GPIO 26/ADC0 (pin 31)-> AOUT or AUD on microphone board
   3.3v (pin 36) -> VCC on microphone board
   GND (pin 38)  -> GND on microphone board
*/
uint adc_raw;

#define ADC_NUM 0
#define ADC_PIN (26 + ADC_NUM)
#define ADC_VREF 3.3
#define ADC_RANGE (1 << 12)
#define ADC_CONVERT (ADC_VREF / (ADC_RANGE - 1))


void setup() {


    bi_decl(bi_program_description("Analog microphone example for Raspberry Pi Pico")); // for picotool
    bi_decl(bi_1pin_with_name(ADC_PIN, "ADC input pin"));

    adc_init();
    adc_gpio_init( ADC_PIN);
    adc_select_input( ADC_NUM);


    pinMode(led, OUTPUT);
    Serial.begin(19200,SERIAL_8N2);
    while (!Serial) {
    }
    delay(2);
    Serial.flush();
    Serial.setTimeout(50);    
    
    Serial.println("Freq_ADC");
 

}

void loop() {
  // put your main code here, to run repeatedly:
  adc_raw = adc_read(); // raw voltage from ADC
  sprintf(hi,"%.2f\n", adc_raw * ADC_CONVERT);
  Serial.print(hi);
  
  //delay(10);

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
