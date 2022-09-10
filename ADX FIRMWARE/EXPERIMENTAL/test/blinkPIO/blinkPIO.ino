#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "blinkPIO.pio.h"


    static const uint led_pin = 3;
    static const float pio_freq = 2000;

    uint sm;
    uint offset;
    PIO  pio;
void setup() {
 
    // Choose PIO instance (0 or 1)
    pio = pio0;

    // Get first free state machine in PIO 0
    //uint sm = pio_claim_unused_sm(pio, true);

    // Add PIO program to PIO instruction memory. SDK will find location and
    // return with the memory offset of the program.
    offset = pio_add_program(pio, &blinkPIO_program);

    // Get first free state machine in PIO 0
    sm = pio_claim_unused_sm(pio, true);

    // Calculate the PIO clock divider
    float div = (float)clock_get_hz(clk_sys) / pio_freq;

    // Initialize the program using the helper function in our .pio file
    blinkPIO_program_init(pio, sm, offset, led_pin, div);

    // Start running our PIO program in the state machine
    pio_sm_set_enabled(pio, sm, true);
    //while (true) ;
}
void loop() {

    // Do nothing
    while (true) {
        //pio_sm_put_blocking(pio, sm, 1);
        sleep_ms(1000);
    }
}
