
// Diagnostic03 
// PDX Project
// Sweep through all 7-bit I2C addresses, to see if any slaves are present on
// the I2C bus. Print out a table that looks like this:
//
// I2C Bus Scan
//   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
// 0
// 1       @
// 2
// 3             @
// 4
// 5
// 6
// 7
//
// E.g. if slave addresses 0x12 and 0x34 were acknowledged.
 
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

char hi[80];
// I2C reserves some addresses for special purposes. We exclude these from the scan.
// These are any addresses of the form 000 0xxx or 111 1xxx
bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}
 
void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.flush();
  delay(50);
  
  // put your setup code here, to run once:
    // Enable UART so we can print status output
#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
#warning i2c/bus_scan example requires a board with I2C pins
    Serial.println("Default I2C pins were not defined");
#else
    // This example will use I2C0 on the default SDA and SCL pins (GP4, GP5 on a Pico)
    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    sprintf(hi,"Default SDA(%d) SCL(%d)\n",int(PICO_DEFAULT_I2C_SDA_PIN),int(PICO_DEFAULT_I2C_SCL_PIN));
    Serial.print(hi);
 
    Serial.println("I2C Bus Scan");
    Serial.println("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F");
 
    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            sprintf(hi,"%02x ", addr);
            Serial.print(hi);
        }
 
        // Perform a 1-byte dummy read from the probe address. If a slave
        // acknowledges this address, the function returns the number of bytes
        // transferred. If the address byte is ignored, the function returns
        // -1.
 
        // Skip over any reserved addresses.
        int ret;
        uint8_t rxdata;
        if (reserved_addr(addr))
            ret = PICO_ERROR_GENERIC;
        else
            ret = i2c_read_blocking(i2c_default, addr, &rxdata, 1, false);

        
        Serial.print(ret < 0 ? "." : "@");
        Serial.print(addr % 16 == 15 ? "\n" : "  ");
    }
    Serial.println("Done.\n");
#endif
}

void loop() {

}
