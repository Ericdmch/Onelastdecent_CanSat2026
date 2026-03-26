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

// ── E220 helpers ──────────────────────────────────────────────────────────

static void e220_flush_rx_verbose() {
    while (uart_is_readable(E220_UART)) {
        uint8_t c = uart_getc(E220_UART);
        printf("  (flushed byte: 0x%02X)\n", c);
    }
}

static void e220_flush_rx() {
    int n = 0;
    while (uart_is_readable(E220_UART)) { uart_getc(E220_UART); n++; }
    if (n) printf("  (flushed %d bytes)\n", n);
}

static int e220_read_resp(uint8_t* buf, int max_len, uint32_t first_byte_us) {
    int len = 0;
    uint32_t timeout = first_byte_us;
    while (len < max_len && uart_is_readable_within_us(E220_UART, timeout)) {
        buf[len++] = uart_getc(E220_UART);
        timeout = 10000; // subsequent bytes: 10ms
    }
    return len;
}

static void e220_print_hex(const char* label, const uint8_t* buf, int len) {
    printf("%s", label);
    for (int i = 0; i < len; i++) printf("%02X ", buf[i]);
    printf("(%d bytes)\n", len);
}

// ── Probe: try BOTH binary and AT protocol ──────────────────────────────

enum ProbeResult { PROBE_NONE, PROBE_BINARY, PROBE_AT };

static ProbeResult e220_probe() {
    uint8_t resp[20];
    int rlen;

    // --- Try binary: C1 00 08 (reference library format) ---
    e220_flush_rx();
    const uint8_t bin_cmd[] = {0xC1, 0x00, 0x08};
    uart_write_blocking(E220_UART, bin_cmd, 3);
    sleep_ms(100);
    rlen = e220_read_resp(resp, sizeof(resp), 100000);
    if (rlen > 0) {
        e220_print_hex("  bin rx: ", resp, rlen);
        if (rlen >= 3 && resp[0] == 0xC1) return PROBE_BINARY;
    }

    // --- Try AT text: AT+ADDR?\r\n ---
    e220_flush_rx();
    const char* at_cmd = "AT+ADDR?\r\n";
    uart_write_blocking(E220_UART, (const uint8_t*)at_cmd, strlen(at_cmd));
    sleep_ms(200);
    rlen = e220_read_resp(resp, sizeof(resp), 100000);
    if (rlen > 0) {
        resp[rlen < 19 ? rlen : 19] = '\0';
        printf("  at rx: \"%s\" (%d bytes)\n", (char*)resp, rlen);
        if (strstr((char*)resp, "OK") || strstr((char*)resp, "AT") || rlen >= 2)
            return PROBE_AT;
    }

    return PROBE_NONE;
}

// ── Config write (binary protocol) ──────────────────────────────────────

static bool e220_write_config_binary() {
    // C0 = permanent save (per reference library; C2 = temporary)
    const uint8_t config[] = {
        0xC0,   // permanent write
        0x00,   // start register
        0x08,   // 8 registers (no NETID in this variant)
        0x00,   // ADDH  = 0
        0x00,   // ADDL  = 0
        0x62,   // SPED: UART 9600, 8N1, air 2.4 kbps
        0x00,   // OPTION: sub-pkt 200B, RSSI off, 30 dBm
        0x41,   // CHAN: channel 65 → 915.125 MHz
        0x00,   // TRANS_MODE: transparent, no relay/LBT/WOR
        0x00,   // CRYPT_H
        0x00,   // CRYPT_L
    };

    e220_flush_rx();
    uart_write_blocking(E220_UART, config, sizeof(config));
    sleep_ms(200);

    uint8_t resp[16];
    int rlen = e220_read_resp(resp, sizeof(resp), 100000);
    if (rlen >= 3 && resp[0] == 0xC1) {
        e220_print_hex("[E220] Write OK (bin): ", resp, rlen);
        return true;
    }
    if (rlen > 0) e220_print_hex("[E220] Write resp: ", resp, rlen);
    return false;
}

// ── Config write (AT protocol) ──────────────────────────────────────────

static bool e220_at_cmd(const char* cmd) {
    e220_flush_rx();
    uart_write_blocking(E220_UART, (const uint8_t*)cmd, strlen(cmd));
    sleep_ms(200);

    char resp[32];
    int rlen = 0;
    while (rlen < (int)sizeof(resp) - 1 && uart_is_readable_within_us(E220_UART, 50000)) {
        uint8_t c = uart_getc(E220_UART);
        if (c >= 0x20 && c < 0x7f) resp[rlen++] = (char)c;
    }
    resp[rlen] = '\0';

    if (rlen > 0) printf("  [%s] -> \"%s\"\n", cmd, resp);
    return strstr(resp, "OK") != nullptr;
}

static void e220_write_config_at() {
    const char* cmds[] = {
        "AT+ADDR=0\r\n",
        "AT+CHANNEL=65\r\n",
        "AT+RATE=2\r\n",
        "AT+TRANS=0\r\n",
        "AT+POWER=0\r\n",
        "AT+UART=3,0\r\n",
        "AT+PACKET=0\r\n",
    };
    for (int i = 0; i < 7; i++) {
        if (!e220_at_cmd(cmds[i]))
            printf("  WARN: %s no OK\n", cmds[i]);
    }
}

// ── E220 configure ──────────────────────────────────────────────────────

static void e220_configure() {
    // --- GPIO: start with M0=HIGH, M1=HIGH (config mode from boot) ---
    gpio_init(E220_M0_PIN);  gpio_set_dir(E220_M0_PIN, GPIO_OUT);  gpio_put(E220_M0_PIN, 1);
    gpio_init(E220_M1_PIN);  gpio_set_dir(E220_M1_PIN, GPIO_OUT);  gpio_put(E220_M1_PIN, 1);
    gpio_init(E220_AUX_PIN); gpio_set_dir(E220_AUX_PIN, GPIO_IN);
    gpio_pull_up(E220_AUX_PIN);

    // --- UART ---
    uint actual_baud = uart_init(E220_UART, E220_BAUD);
    gpio_set_function(E220_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(E220_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(E220_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(E220_UART, true);

    printf("[E220] UART baud=%u actual=%u\n", E220_BAUD, actual_baud);
    printf("[E220] Pins: TX=GP%d RX=GP%d M0=GP%d M1=GP%d AUX=GP%d\n",
           E220_TX_PIN, E220_RX_PIN, E220_M0_PIN, E220_M1_PIN, E220_AUX_PIN);

    // --- Line-level check ---
    gpio_set_function(E220_RX_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(E220_RX_PIN, GPIO_IN);
    gpio_disable_pulls(E220_RX_PIN);
    sleep_ms(5);
    bool rx_level = gpio_get(E220_RX_PIN);
    printf("[E220] GP1 (RX) no-pull=%d %s\n", rx_level,
           rx_level ? "(E220 TXD connected)" : "*** NOT CONNECTED ***");
    gpio_set_function(E220_RX_PIN, GPIO_FUNC_UART);

    // --- Wait for AUX HIGH ---
    printf("[E220] Waiting for AUX HIGH (config mode)...\n");
    for (int i = 0; i < 300; i++) {
        if (gpio_get(E220_AUX_PIN)) break;
        if (i % 50 == 0) printf("  AUX=%d @%dms\n", gpio_get(E220_AUX_PIN), i * 10);
        sleep_ms(10);
    }
    sleep_ms(200);

    // Print the stale byte if there is one
    e220_flush_rx_verbose();

    printf("[E220] M0=%d M1=%d AUX=%d — probing...\n",
           gpio_get(E220_M0_PIN), gpio_get(E220_M1_PIN), gpio_get(E220_AUX_PIN));

    // --- Probe at multiple baud rates and all M0/M1 mode combos ---
    const uint32_t bauds[] = {9600, 115200, 57600, 19200, 38400, 4800, 2400, 1200};
    const int num_bauds = sizeof(bauds) / sizeof(bauds[0]);

    ProbeResult result = PROBE_NONE;

    // First: try 9600 with M0=1 M1=1 (standard config mode), 5 quick probes
    printf("[E220] === 9600 baud, M0=1 M1=1 (5 probes) ===\n");
    for (int i = 0; i < 5 && result == PROBE_NONE; i++) {
        printf("  [probe %d] ", i + 1);
        result = e220_probe();
        if (result == PROBE_NONE) { printf("  no response\n"); sleep_ms(300); }
    }

    // Second: try all other baud rates with M0=1 M1=1
    if (result == PROBE_NONE) {
        for (int b = 1; b < num_bauds && result == PROBE_NONE; b++) {
            uart_set_baudrate(E220_UART, bauds[b]);
            printf("[E220] === %lu baud, M0=1 M1=1 ===\n", (unsigned long)bauds[b]);
            e220_flush_rx_verbose();
            for (int i = 0; i < 3 && result == PROBE_NONE; i++) {
                printf("  [probe %d] ", i + 1);
                result = e220_probe();
                if (result == PROBE_NONE) { printf("  no response\n"); sleep_ms(200); }
            }
        }
        // Restore 9600
        uart_set_baudrate(E220_UART, 9600);
    }

    // Third: try other M0/M1 combos at 9600
    if (result == PROBE_NONE) {
        // M0=0 M1=1
        printf("[E220] === 9600 baud, M0=0 M1=1 ===\n");
        gpio_put(E220_M0_PIN, 0); gpio_put(E220_M1_PIN, 1);
        sleep_ms(500);
        for (int i = 0; i < 3 && result == PROBE_NONE; i++) {
            printf("  [probe %d] ", i + 1);
            result = e220_probe();
            if (result == PROBE_NONE) { printf("  no response\n"); sleep_ms(200); }
        }

        // M0=1 M1=0
        printf("[E220] === 9600 baud, M0=1 M1=0 ===\n");
        gpio_put(E220_M0_PIN, 1); gpio_put(E220_M1_PIN, 0);
        sleep_ms(500);
        for (int i = 0; i < 3 && result == PROBE_NONE; i++) {
            printf("  [probe %d] ", i + 1);
            result = e220_probe();
            if (result == PROBE_NONE) { printf("  no response\n"); sleep_ms(200); }
        }

        // Restore config mode
        gpio_put(E220_M0_PIN, 1); gpio_put(E220_M1_PIN, 1);
        sleep_ms(500);
    }

    if (result == PROBE_NONE) {
        printf("\n[E220] Exhausted all baud rates and mode combos. No response.\n");
        printf("  The module is connected (TXD drives GP1 HIGH, AUX HIGH)\n");
        printf("  but does not respond to any command.\n");
        printf("[E220] Continuing without config — using factory defaults.\n");
    }

    // --- Write configuration using whichever protocol worked ---
    if (result == PROBE_BINARY) {
        printf("[E220] Using binary protocol\n");
        if (!e220_write_config_binary())
            printf("[E220] WARN: binary write failed\n");
    } else if (result == PROBE_AT) {
        printf("[E220] Using AT protocol\n");
        e220_write_config_at();
    }
    // result == PROBE_NONE: skip config, use factory defaults

    // --- Return to normal mode ---
    gpio_put(E220_M0_PIN, 0);
    gpio_put(E220_M1_PIN, 0);
    sleep_ms(500);
    for (int i = 0; i < 100; i++) {
        if (gpio_get(E220_AUX_PIN)) break;
        sleep_ms(10);
    }
    printf("[E220] Normal mode. AUX=%d — Ready\n", gpio_get(E220_AUX_PIN));
}

int main() {
    stdio_init_all();
    sleep_ms(3000);  // wait for serial monitor to connect

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    if (watchdog_caused_reboot()) {
        printf("[BOOT] Watchdog reboot\n");
    }

    // Configure E220 (handles all GPIO/UART init internally)
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
