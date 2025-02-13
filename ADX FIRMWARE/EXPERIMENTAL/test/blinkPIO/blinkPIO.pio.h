// -------------------------------------------------- //
// This file is autogenerated by pioasm; do not edit! //
// -------------------------------------------------- //

#pragma once

#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// -------- //
// blinkPIO //
// -------- //

#define blinkPIO_wrap_target 0
#define blinkPIO_wrap 18

static const uint16_t blinkPIO_program_instructions[] = {
            //     .wrap_target
    0xe040, //  0: set    y, 0                       
    0xe03e, //  1: set    x, 30                      
    0x006a, //  2: jmp    !y, 10                     
    0xf300, //  3: set    pins, 0                [19]
    0xb342, //  4: nop                           [19]
    0xb342, //  5: nop                           [19]
    0xb342, //  6: nop                           [19]
    0xb342, //  7: nop                           [19]
    0x0040, //  8: jmp    x--, 0                     
    0x0002, //  9: jmp    2                          
    0xf301, // 10: set    pins, 1                [19]
    0xb342, // 11: nop                           [19]
    0xb342, // 12: nop                           [19]
    0xb342, // 13: nop                           [19]
    0xb342, // 14: nop                           [19]
    0x0051, // 15: jmp    x--, 17                    
    0x0002, // 16: jmp    2                          
    0xe041, // 17: set    y, 1                       
    0x0001, // 18: jmp    1                          
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program blinkPIO_program = {
    .instructions = blinkPIO_program_instructions,
    .length = 19,
    .origin = -1,
};

static inline pio_sm_config blinkPIO_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + blinkPIO_wrap_target, offset + blinkPIO_wrap);
    return c;
}

// Helper function (for use in C program) to initialize this PIO program
void blinkPIO_program_init(PIO pio, uint sm, uint offset, uint pin, float div) {
    // Sets up state machine and wrap target. This function is automatically
    // generated in blink.pio.h.
    pio_sm_config c = blinkPIO_program_get_default_config(offset);
    // Allow PIO to control GPIO pin (as output)
    pio_gpio_init(pio, pin);
    // Connect pin to SET pin (control with 'set' instruction)
    sm_config_set_set_pins(&c, pin, 1);
    // Set the pin direction to output (in PIO)
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
    // Set the clock divider for the state machine
    sm_config_set_clkdiv(&c, div);
    // Load configuration and jump to start of the program
    pio_sm_init(pio, sm, offset, &c);
}

#endif

