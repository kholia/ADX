/**
 * MIT License
 * 
 * Copyright (c) 2022 Daniel Garcia-Briseno
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

 /*-----------------------------------------------------------------------------
  * freqPIO
  * This program is a WIP product for testing purposes of the PDX projects
  * 
  * It's purpose is to evaluate the accuracy of the PIO driven logic to meassure
  * frequency on a FT8 protocol transmiter
  * Code inspired on simply_isr by Daniel Garcia Briceno
  * Code modified and evaluated by Dr. Pedro E. Colla (LU7DZ)
  * 
  */

#include <stdio.h>
#include "pico/stdlib.h"

#include "freqPIO.pio.h"
#include "pico.h"
#include "hardware/structs/pio.h"

/**
 * These defines represent each state machine.
 * The value is the bit in the IRQ register that
 * will be set by each state machine thanks to "irq wait 0 rel"
 */
#define PIO_SM_0_IRQ 0b0001
#define PIO_SM_1_IRQ 0b0010
#define PIO_SM_2_IRQ 0b0100
#define PIO_SM_3_IRQ 0b1000

#define GPIO27              27    //Pin where the signal will be
#define FSKMIN             300    //Minimum FSK frequency computed
#define FSKMAX            3000    //Maximum FSK frequency computed


/**
 * This variable will shadow the IRQ flags set by the PIO state machines.
 * Typically you do not want to do work in ISRs because the main thread
 * has more important things to do. Because of that, when we get the ISR
 * I'm simply going to copy the state machine that fired the ISR into
 * this variable.
 *
 * Variable is volatile so that it doesn't get cached in a CPU register
 * in the main thread. Without this it's possible that you never see
 * irq_flags get set even though the ISR is firing all the time.
 *
 * Of course, you can really do whatever you want in the ISR, it's up to you.
 */
volatile uint32_t   irq_flags    = 0;
volatile bool       flagIRQ      = false;
volatile uint32_t   n            = 0;
char hi[80];
static const        uint TX      = 3;
volatile bool       led_lite     = true;
volatile uint32_t   t_previous;
volatile uint32_t   t_current;
volatile uint32_t   period;


/**
 * This function is called when the IRQ is fired by the state machine.
 * @note See enable_pio_isrs for how to register this function to be called
 */
void simply_isr_handler() {
    // Read the IRQ register to get the IRQ flags from the state machine
    // This tells me which state machine sent the IRQ
    
    irq_flags = pio0_hw->irq;
    flagIRQ   = true;
    n++;
    t_current = time_us_32();
    period=t_current-t_previous;
    t_previous=t_current;
    
    
    // IRQ_OFFSET is write 1 to clear, so by writing back the
    // value, we're acknowledging that we've serviced the interrupt.
    
    hw_clear_bits(&pio0_hw->irq, irq_flags);
}

/**
 * Lets the pico know that we want it to notify us of the PIO ISRs.
 * @note in simply_isr.pio we enable irq0. This tells the state machine
 *       to send the ISRs to the core, we still need to tell the core
 *       to send them to our program.
 */
void enable_pio_isrs() {
    // Set the function that will be called when the PIO IRQ comes in.
    
    irq_set_exclusive_handler(PIO0_IRQ_0, simply_isr_handler);

    // Once that function is set, we can go ahead and allow the interrupts
    // to come in. You want to set the function before enabling the interrupt
    // just in case. The docs say if an IRQ comes in and there's no handler
    // then it will work like a breakpoint, which seems bad.
    
    irq_set_enabled(PIO0_IRQ_0, true);
}

/**
 * Loads simply_isr pio program into PIO memory
 */
void load_pio_programs() {
    
    PIO pio = pio0;

    // Load the program into PIO memory
    uint offset = pio_add_program(pio, &freqPIO_program);

    // Load the program to run in each state machine.
    // They are allowed to run the same program in memory.
    // Pass the pin where the signal will be
    
    freqPIO_program_init(pio, 0, offset,GPIO27);
}

/**
 * Writes to the tx fifo of the given state machine.
 * This will make the simply_isr program send an ISR to us!
 */
void trigger_isr(int sm) {
    
    sprintf(hi,"Triggering ISR from state machine %d\n", sm);
    Serial.print(hi);
    
    pio_sm_put_blocking(pio0, sm, 1);
    
    // ISR will fire from the pio right here thanks to above function.

    // Print the irq we expect based on the given state machine
    sprintf(hi,"Expected IRQ flags: 0x%08X\n", (1 << sm));
    Serial.print(hi);
    sprintf(hi,"Actual IRQ Flags: 0x%08X\n", irq_flags);
    Serial.print(hi);

    // Here you could do work for the isr depending on which one it is.
    // Something like
    
    if (irq_flags & PIO_SM_0_IRQ) {
        // handle_sm0_irq();
    }
    
    // clear irq flags now.
    irq_flags = 0;
}

/*-------------------------------------------------------------------------*
 * setup()
 * 
 */
void setup() {

    Serial.begin(115200);
    while (!Serial) {}
    Serial.println("FreqPIO PIO freq counter test firmware ready");

/*-------
 * Define and Turn off the TX lit
 */
    gpio_init(TX);
    gpio_set_dir(TX, GPIO_OUT);

    led_lite=!led_lite;
    gpio_put(TX,led_lite);

    // Load FreqPIO microcode for the PIO sequential machine into memory
    load_pio_programs();

    // Enable IRQs to respond to FreqPIO
    enable_pio_isrs();
}

/*--------------
 * loop()
 * Just wait for IRQ request made by the PIO sequential machine signaling
 * a complete cycle measurement
 */
void loop() {
  
    // the IRQ is fired by the PIO asm microcode, each time a full cycle
    // has been measured the result is communicated.

    while (true) {

        if (flagIRQ == true) {
           flagIRQ = false;
           uint32_t  f=1000000;
           double ffsk=0.0;
           
           if (period>0) {
              
              f=f/(period);
              ffsk = double(1000000.0) / double(period);               //Ticks are expressed in uSecs so convert to Hz
              ffsk = round(ffsk);                        
              
              sprintf(hi,"IRQ generated  T=%ld f(int)=%ld Hz  f(float)=%10.2f Hz\n",period+1,f,ffsk);
              Serial.print(hi);

           } else {
              f=0;   
           }
           
           n=0;
           led_lite=!led_lite;
           gpio_put(TX,led_lite);
        }
    }
}
