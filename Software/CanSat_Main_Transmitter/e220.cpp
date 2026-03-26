#include "e220.hpp"
#include "pico/stdlib.h"
#include <cstdio>
#include <cstring>

// Flush any stale bytes from the RX FIFO
static void e220_flush_rx() {
    while (uart_is_readable(E220_UART)) uart_getc(E220_UART);
}

// Send one AT command, retry once on no-response. Returns true on =OK.
static bool e220_at_send(const char* cmd) {
    char display[64];
    strncpy(display, cmd, sizeof(display) - 1);
    display[sizeof(display) - 1] = '\0';
    for (char* p = display; *p; p++) {
        if (*p == '\r' || *p == '\n') { *p = '\0'; break; }
    }
    printf("  >> %s\n", display);

    for (int attempt = 0; attempt < 2; attempt++) {
        e220_flush_rx();
        uart_write_blocking(E220_UART, (const uint8_t*)cmd, strlen(cmd));
        sleep_ms(200);

        char resp[32];
        int  rlen = 0;
        while (rlen < (int)sizeof(resp) - 1 &&
               uart_is_readable_within_us(E220_UART, 10000)) {
            char c = uart_getc(E220_UART);
            if (c != '\r' && c != '\n') resp[rlen++] = c;
        }
        resp[rlen] = '\0';

        bool ok = (strstr(resp, "=OK") != nullptr);
        if (rlen > 0) {
            printf("  << %s%s\n", resp, ok ? "" : "  [!]");
            return ok;
        }
        // No response — wait longer before retry
        sleep_ms(500);
    }
    printf("  << (no response)  [!]\n");
    return false;
}

void E220::init() {
    // M0 and M1 as outputs, both LOW → Normal (Transmission) Mode
    gpio_init(E220_M0_PIN);  gpio_set_dir(E220_M0_PIN, GPIO_OUT);  gpio_put(E220_M0_PIN, 0);
    gpio_init(E220_M1_PIN);  gpio_set_dir(E220_M1_PIN, GPIO_OUT);  gpio_put(E220_M1_PIN, 0);

    // AUX as input with pull-up: if wire is disconnected it reads HIGH (safe),
    // if module is connected it will drive it LOW when busy (overrides pull-up).
    gpio_init(E220_AUX_PIN); gpio_set_dir(E220_AUX_PIN, GPIO_IN);
    gpio_pull_up(E220_AUX_PIN);

    // UART0 for E220
    uart_init(E220_UART, E220_BAUD);
    gpio_set_function(E220_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(E220_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(E220_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(E220_UART, true);

    // Wait up to 3 s for AUX to go HIGH
    for (int i = 0; i < 300; i++) {
        if (ready()) break;
        sleep_ms(10);
    }

    printf("[E220] AUX=%d M0=0 M1=0 (normal mode baseline)\n", gpio_get(E220_AUX_PIN));

    // Enter config mode (M1=1, M0=1) and apply AT settings
    printf("[E220] Entering config mode\n");
    gpio_put(E220_M1_PIN, 1);
    gpio_put(E220_M0_PIN, 1);
    sleep_ms(500); // initial settle before polling

    // Probe with a real command until the module responds (up to 30 s)
    printf("[E220] Waiting for config mode... AUX=%d\n", gpio_get(E220_AUX_PIN));
    bool cfg_ok = false;
    for (int i = 0; i < 60; i++) {
        e220_flush_rx();
        uart_write_blocking(E220_UART, (const uint8_t*)"AT+ADDR=0\r\n", 11);
        sleep_ms(400);

        char resp[16]; int rlen = 0;
        while (rlen < (int)sizeof(resp) - 1 && uart_is_readable_within_us(E220_UART, 10000)) {
            uint8_t c = uart_getc(E220_UART);
            if (c >= 0x20 && c < 0x7f) resp[rlen++] = (char)c;
        }
        resp[rlen] = '\0';

        if (strstr(resp, "OK") != nullptr) {
            printf("[E220] Config mode ready (attempt %d) — AT+ADDR=0 =OK\n", i + 1);
            cfg_ok = true;
            break;
        }
        printf("  [probe %2d] AUX=%d resp=\"%s\"\n", i + 1, gpio_get(E220_AUX_PIN), resp);
        sleep_ms(100);
    }
    if (!cfg_ok) printf("[E220] WARNING: module not responding — sending config anyway\n");

    cfg_ok &= e220_at_send("AT+CHANNEL=65\r\n");
    cfg_ok &= e220_at_send("AT+RATE=2\r\n");
    cfg_ok &= e220_at_send("AT+TRANS=0\r\n");
    cfg_ok &= e220_at_send("AT+POWER=0\r\n");
    cfg_ok &= e220_at_send("AT+UART=3,0\r\n");
    cfg_ok &= e220_at_send("AT+PACKET=0\r\n");
    cfg_ok &= e220_at_send("AT+UAUX=1\r\n");

    if (!cfg_ok) {
        printf("[E220] WARNING: config incomplete — continuing with factory defaults\n");
    } else {
        printf("[E220] Config OK\n");
    }

    // Return to normal mode (M1=0, M0=0)
    gpio_put(E220_M1_PIN, 0);
    gpio_put(E220_M0_PIN, 0);
    sleep_ms(2000);

    printf("[E220] Ready\n");
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
