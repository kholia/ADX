/*
  From (https://arduino-pico.readthedocs.io/en/latest/multicore.html)
  
  By adding a setup1() and loop1() function to your sketch you can make use of the second core. 
  Anything called from within the setup1() or loop1() routines will execute on the second core.

  Functions:
  1) Pausing Cores
  - void rp2040.idleOtherCore();
  - void rp2040.resumeOtherCore();

  2) Communicating Between Cores
  - void rp2040.fifo.push(uint32_t);
  - bool rp2040.fifo.push_nb(uint32_t);
  - bool rp2040.fifo.push_nb(uint32_t);
  - bool rp2040.fifo.pop_nb(uint32_t *dest);
  - int rp2040.fifo.available();
*/

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"

#define FLAG_VALUE    0xDEADBEEF

void heartBeatPrint(bool isCore0 = true)
{
  static int num = 1;

  Serial.print(isCore0 ? F("C0") : F("C1") );

  if (num == 40)
  {
    Serial.println();
    num = 1;
  }
  else if (num++ % 10 == 0)
  {
    Serial.print(F(" "));
  }
}

void check_status(bool isCore0 = true)
{
  static unsigned long checkstatus_timeout = 0;

#define STATUS_CHECK_INTERVAL     10000L

  // Send status report every STATUS_REPORT_INTERVAL (60) seconds: we don't need to send updates frequently if there is no status change.
  if ((millis() > checkstatus_timeout) || (checkstatus_timeout == 0))
  {
    heartBeatPrint(isCore0);
    checkstatus_timeout = millis() + STATUS_CHECK_INTERVAL;
  }
}

//////////////////////////////////////////////

// Running on Core0

void setup()
{
  rp2040.idleOtherCore();

  Serial.begin(115200);
  while (!Serial);

  Serial.print("\nStart RPI_Pico_Multicore Core0 on ");
  Serial.println(BOARD_NAME);

  rp2040.resumeOtherCore();

  uint32_t g = rp2040.fifo.pop();

  rp2040.idleOtherCore();

  Serial.print("C0: rp2040_fifo_pop g = 0x");
  Serial.println(g, HEX);

  if (g == FLAG_VALUE)
  {
    rp2040.fifo.push(FLAG_VALUE);
    Serial.println("Core0 OK!");
  }
  else
    Serial.println("Core0 not OK!");

  rp2040.resumeOtherCore();
}

void loop()
{
  check_status(true);
}

//////////////////////////////////////////////

// Running on Core1

void core1_entry()
{
  rp2040.fifo.push(FLAG_VALUE);

  uint32_t g = rp2040.fifo.pop();

  rp2040.idleOtherCore();

  Serial.print("C1: rp2040_fifo_pop g = 0x");
  Serial.println(g, HEX);

  if (g == FLAG_VALUE)
    Serial.println("Core1 OK!");
  else
    Serial.println("Core1 not OK!");

  rp2040.resumeOtherCore();
}

void setup1()
{
  rp2040.idleOtherCore();

  Serial.begin(115200);
  while (!Serial);

  Serial.print("Start RPI_Pico_Multicore Core1 on ");
  Serial.println(BOARD_NAME);

  rp2040.resumeOtherCore();

  core1_entry();
}

void loop1()
{
  check_status(false);
}
