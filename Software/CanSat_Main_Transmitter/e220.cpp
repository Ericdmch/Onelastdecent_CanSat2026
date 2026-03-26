#include "e220.hpp"
#include "pico/stdlib.h"
#include <cstring>

void E220::init() {
    // M0 and M1 as outputs, both LOW → Normal (Transmission) Mode
    gpio_init(E220_M0_PIN);  gpio_set_dir(E220_M0_PIN, GPIO_OUT);  gpio_put(E220_M0_PIN, 0);
    gpio_init(E220_M1_PIN);  gpio_set_dir(E220_M1_PIN, GPIO_OUT);  gpio_put(E220_M1_PIN, 0);

    // AUX as input (no pull — the module drives this line)
    gpio_init(E220_AUX_PIN); gpio_set_dir(E220_AUX_PIN, GPIO_IN);

    // UART0 for E220
    uart_init(E220_UART, E220_BAUD);
    gpio_set_function(E220_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(E220_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(E220_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(E220_UART, true);

    // Wait up to 3 s for AUX to go HIGH (E220 cold-boot can take several hundred ms).
    // Polling here avoids prematurely cycling M1 while the module is still booting.
    for (int i = 0; i < 300; i++) {
        if (ready()) return;
        sleep_ms(10);
    }

    // AUX still LOW after 3 s — module is stuck in the wrong mode.
    // Cycle M1 HIGH→LOW to force it back to Normal mode.
    gpio_put(E220_M1_PIN, 1);
    sleep_ms(100);
    gpio_put(E220_M1_PIN, 0);
    sleep_ms(500);
}

bool E220::ready() const {
    return gpio_get(E220_AUX_PIN) == 1;
}

bool E220::transmit(const uint8_t* data, size_t len, uint32_t timeout_ms) {
    // Poll AUX until HIGH or timeout
    uint32_t start = to_ms_since_boot(get_absolute_time());
    while (!ready()) {
        if (to_ms_since_boot(get_absolute_time()) - start >= timeout_ms)
            return false;
        sleep_ms(1);
    }

    uart_write_blocking(E220_UART, data, len);
    return true;
}

bool E220::transmit_str(const char* s, uint32_t timeout_ms) {
    return transmit(reinterpret_cast<const uint8_t*>(s), strlen(s), timeout_ms);
}
