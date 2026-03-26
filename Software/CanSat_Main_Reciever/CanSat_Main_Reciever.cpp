//
// CanSat 2026 — Ground Station Receiver Firmware
// Controller : Raspberry Pi Pico 2 (RP2350 Cortex-M33)
// Radio      : E220-900T30D via UART0 (TX=GP0, RX=GP1, M0=GP2, M1=GP3, AUX=GP6)
// Output     : USB serial — open at any baud rate in a serial monitor
//
// Received packets are forwarded verbatim to USB:
//   $CANSAT,<ms>,<temp>,<press>,<hum>,<gas>,<lat_e7>,<lon_e7>,<alt_mm>,<roll>,<pitch>,<yaw>
//

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include <cstdint>
#include <cstdio>
#include <cstring>

static constexpr uint LED_PIN = 25;

// ── E220 pin assignments ───────────────────────────────────────────────────
#define E220_UART      uart0
static constexpr uint E220_TX_PIN  = 0;
static constexpr uint E220_RX_PIN  = 1;
static constexpr uint E220_M0_PIN  = 2;
static constexpr uint E220_M1_PIN  = 3;
static constexpr uint E220_AUX_PIN = 6;
static constexpr uint E220_BAUD    = 9600;

// ── Watchdog ───────────────────────────────────────────────────────────────
static constexpr uint32_t WATCHDOG_MS = 8000;

// ── Packet buffer ─────────────────────────────────────────────────────────
static constexpr size_t BUF_SIZE = 256;

// ── E220 AT configuration ─────────────────────────────────────────────────
// Must be called after UART and GPIO are initialised, while AUX is HIGH.
// Enters config mode (M1=1, M0=1), sends AT commands, returns to normal mode.
// Returns true if response contains "=OK", false if "=ERR" or no response
static bool e220_at_send(const char* cmd) {
    // Print command (strip trailing \r\n for clean output)
    char display[64];
    strncpy(display, cmd, sizeof(display) - 1);
    display[sizeof(display) - 1] = '\0';
    for (char* p = display; *p; p++) {
        if (*p == '\r' || *p == '\n') { *p = '\0'; break; }
    }
    printf("  >> %s\n", display);

    uart_write_blocking(E220_UART, (const uint8_t*)cmd, strlen(cmd));
    sleep_ms(100);

    // Read and print response
    char resp[32];
    int  rlen = 0;
    while (rlen < (int)sizeof(resp) - 1 &&
           uart_is_readable_within_us(E220_UART, 5000)) {
        char c = uart_getc(E220_UART);
        if (c != '\r' && c != '\n') resp[rlen++] = c;
    }
    resp[rlen] = '\0';

    bool ok = (strstr(resp, "=OK") != nullptr);
    if (rlen > 0) {
        printf("  << %s%s\n", resp, ok ? "" : "  [!]");
    } else {
        printf("  << (no response)  [!]\n");
    }
    return ok;
}

static bool e220_configure_attempt() {
    gpio_put(E220_M1_PIN, 1);
    gpio_put(E220_M0_PIN, 1);
    sleep_ms(20);
    for (int i = 0; i < 100; i++) {
        if (gpio_get(E220_AUX_PIN)) break;
        sleep_ms(10);
    }

    bool ok = true;
    ok &= e220_at_send("AT+ADDR=0\r\n");
    ok &= e220_at_send("AT+CHANNEL=65\r\n");
    ok &= e220_at_send("AT+RATE=2\r\n");
    ok &= e220_at_send("AT+TRANS=0\r\n");
    ok &= e220_at_send("AT+POWER=0\r\n");
    ok &= e220_at_send("AT+UART=3,0\r\n");
    ok &= e220_at_send("AT+PACKET=0\r\n");
    ok &= e220_at_send("AT+UAUX=1\r\n");

    gpio_put(E220_M1_PIN, 0);
    gpio_put(E220_M0_PIN, 0);
    sleep_ms(20);
    for (int i = 0; i < 100; i++) {
        if (gpio_get(E220_AUX_PIN)) break;
        sleep_ms(10);
    }
    return ok;
}

static void e220_configure() {
    for (int attempt = 1; attempt <= 3; attempt++) {
        printf("[E220] Config attempt %d/3\n", attempt);
        if (e220_configure_attempt()) {
            printf("[E220] Config OK\n");
            return;
        }
        printf("[E220] Attempt %d failed — retrying\n", attempt);
        sleep_ms(500);
    }
    panic("[E220] Config failed after 3 attempts — check wiring");
}

int main() {
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    if (watchdog_caused_reboot()) {
        printf("[BOOT] Watchdog reboot\n");
    }

    // --- E220: M0/M1 LOW = Normal (transparent) mode
    gpio_init(E220_M0_PIN);  gpio_set_dir(E220_M0_PIN, GPIO_OUT);  gpio_put(E220_M0_PIN, 0);
    gpio_init(E220_M1_PIN);  gpio_set_dir(E220_M1_PIN, GPIO_OUT);  gpio_put(E220_M1_PIN, 0);
    gpio_init(E220_AUX_PIN); gpio_set_dir(E220_AUX_PIN, GPIO_IN);

    uart_init(E220_UART, E220_BAUD);
    gpio_set_function(E220_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(E220_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(E220_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(E220_UART, true);

    // Wait up to 3 s for E220 AUX HIGH (module ready)
    for (int i = 0; i < 300; i++) {
        if (gpio_get(E220_AUX_PIN)) break;
        sleep_ms(10);
    }
    // If still LOW, module booted in wrong mode — cycle M1 to recover
    if (!gpio_get(E220_AUX_PIN)) {
        gpio_put(E220_M1_PIN, 1);
        sleep_ms(100);
        gpio_put(E220_M1_PIN, 0);
        sleep_ms(500);
    }

    e220_configure();

    watchdog_enable(WATCHDOG_MS, true);
    printf("[OK] Ground station ready — waiting for packets\n");
    gpio_put(LED_PIN, 1); // LED solid = ready

    // ── Receive loop ──────────────────────────────────────────────────────
    char line[BUF_SIZE];
    int  pos = 0;
    uint32_t last_heartbeat_ms = 0;

    while (true) {
        watchdog_update();

        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_heartbeat_ms >= 2000) {
            printf("[SCAN] Listening for CanSat...\n");
            last_heartbeat_ms = now;
        }

        if (!uart_is_readable(E220_UART)) {
            sleep_us(200);
            continue;
        }

        char c = uart_getc(E220_UART);

        if (c == '\n') {
            line[pos] = '\0';
            if (pos > 0 && line[0] == '$') {
                // Forward complete packet to USB
                printf("%s\n", line);
                // Brief LED blink to show packet received
                gpio_put(LED_PIN, 0);
                sleep_ms(30);
                gpio_put(LED_PIN, 1);
            }
            pos = 0;
        } else if (c != '\r') {
            if (pos < (int)sizeof(line) - 1) {
                line[pos++] = c;
            } else {
                pos = 0; // line overflow — discard and reset
            }
        }
    }
}
