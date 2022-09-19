/**
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

    #define WIFI_SSID "FiberTel WiFi996 2.4GHz"
    #define WIFI_PASSWORD "00413322447"

//int main() {
void setup() {
    //stdio_init_all();
    if (cyw43_arch_init()) {
        printf("WiFi init failed");
        return;
    }
    while (true) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        sleep_ms(250);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(250);
    }
}
void loop() {

}
