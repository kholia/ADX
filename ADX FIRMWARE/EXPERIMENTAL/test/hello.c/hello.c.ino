/****************************
hello.c.ino
****************************/

#include <hardware/pio.h>

// Our assembled program:
#include "hello.pio.h"
#define TX 3

PIO pio = pio0;
uint offset;
uint sm;

void setup() {
  offset = pio_add_program(pio, &hello_program);
  sm = pio_claim_unused_sm(pio, true);
  hello_program_init(pio, sm, offset, TX);
}
void dit() {
  pio_sm_put_blocking(pio, sm, 1);
  sleep_ms(200);
  pio_sm_put_blocking(pio, sm, 0);
  sleep_ms(200);
  
}
void dah() {
  pio_sm_put_blocking(pio, sm, 1);
  sleep_ms(200);
  pio_sm_put_blocking(pio, sm, 1);
  sleep_ms(200);
  pio_sm_put_blocking(pio, sm, 1);
  sleep_ms(200);
  pio_sm_put_blocking(pio, sm, 0);
  sleep_ms(200);
  
}
void loop() {
  // put your main code here, to run repeatedly:
  dit();
  dit();
  dit();
  sleep_ms(200);
  dah();
  dah();
  dah();
  sleep_ms(200);
  dit();
  dit();
  dit();
  sleep_ms(600);
}
