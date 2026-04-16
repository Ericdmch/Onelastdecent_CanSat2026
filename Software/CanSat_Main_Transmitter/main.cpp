//
// CanSat 2026 — Primary Mission Firmware
// Controller : Raspberry Pi Pico 2 W (RP2350 Cortex-M33)
// Sensor     : BME680  via I2C0  (SDA=GP4, SCL=GP5)
// Radio      : E220-900T30D via UART0 (TX=GP0, RX=GP1, M0=GP2, M1=GP3, AUX=GP6)
// FC Link    : MicoAir743V2 (ArduPilot) via UART1 (TX=GP8, RX=GP9)
// NeoPixel   : WS2812B via PIO (DIN=GP7)
//

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"
#include "hardware/clocks.h"

#include "bme680.hpp"
#include "e220.hpp"
#include "fc_link.hpp"
#include "neopixel.hpp"

#include <cstdio>
#include <cstring>

// ── Pin assignments ────────────────────────────────────────────────────────
static constexpr uint I2C0_SDA = 4;
static constexpr uint I2C0_SCL = 5;

// ── Calibration ────────────────────────────────────────────────────────────
static constexpr float TEMP_OFFSET_C = -6.51f; // corrects for PCB self-heating

// ── Radio frequency ────────────────────────────────────────────────────────
static constexpr uint E220_CHANNEL  = 65;
static constexpr float   E220_FREQ_MHZ = 850.125f + E220_CHANNEL; // 915.125 MHz

// ── Timing ─────────────────────────────────────────────────────────────────
static constexpr uint32_t LOOP_PERIOD_MS  = 1000; // 1 Hz telemetry
static constexpr uint32_t WATCHDOG_MS     = 2000; // reboot if loop hangs

// ── Globals ────────────────────────────────────────────────────────────────
static BME680  bme(i2c0);
static E220    radio;
static FCLink  fc;

// ── Helpers ────────────────────────────────────────────────────────────────
static void led_blink(uint n) {
    for (uint i = 0; i < n; i++) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1); sleep_ms(80);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0); sleep_ms(80);
    }
}

// Build CSV telemetry packet into buf; returns number of chars written
// Format: $CANSAT,<ms>,<temp>,<press>,<hum>,<gas>,<lat_e7>,<lon_e7>,<alt_mm>,<roll_deg>,<pitch_deg>,<yaw_deg>\r\n
static int build_packet(char* buf, size_t bufsz,
                        uint32_t timestamp_ms,
                        const BME680Data& env,
                        const FCData& fc_d,
                        uint32_t fc_bytes,
                        uint32_t stx_v1,
                        uint32_t stx_v2)
{
    const float R2D = 57.2957795f;

    int n = snprintf(buf, bufsz,
        "$CANSAT,%lu,"
        "%.2f,%.2f,%.2f,%.0f,"
        "%ld,%ld,%ld,"
        "%.1f,%.1f,%.1f,"
        "%lu,%lu,%lu"
        "\r\n",
        (unsigned long)timestamp_ms,
        // BME680
        env.valid ? env.temperature + TEMP_OFFSET_C : -999.0f,
        env.valid ? env.pressure      : -999.0f,
        env.valid ? env.humidity      : -999.0f,
        env.valid ? env.gas_resistance: -999.0f,
        // GPS
        fc_d.gps_valid ? (long)fc_d.lat    : 0L,
        fc_d.gps_valid ? (long)fc_d.lon    : 0L,
        fc_d.gps_valid ? (long)fc_d.alt_mm : 0L,
        // Attitude (convert rad→deg)
        fc_d.att_valid ? fc_d.roll  * R2D : 0.0f,
        fc_d.att_valid ? fc_d.pitch * R2D : 0.0f,
        fc_d.att_valid ? fc_d.yaw   * R2D : 0.0f,
        // Raw FC UART byte counter and MAVLink version probe (debug)
        (unsigned long)fc_bytes,
        (unsigned long)stx_v1,
        (unsigned long)stx_v2
    );
    return n;
}

// ── Entry point ────────────────────────────────────────────────────────────
int main() {
    // --- USB stdio for debug output (UART0 reserved for E220)
    stdio_init_all();
    sleep_ms(2000); // allow time to open serial monitor before config output begins

    // --- CYW43 (required for onboard LED on Pico W)
    cyw43_arch_init();

    // --- NeoPixel
    neopixel_init();

    if (watchdog_caused_reboot()) {
        printf("[BOOT] Watchdog reboot detected\n");
    }

    // --- I2C0 for BME680 (400 kHz)
    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(I2C0_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA);
    gpio_pull_up(I2C0_SCL);

    // --- Peripherals
    radio.init();

    printf("[E220] TX frequency: %.3f MHz (channel %u)\n", (double)E220_FREQ_MHZ, E220_CHANNEL);
    {
        char freq_msg[48];
        int n = snprintf(freq_msg, sizeof(freq_msg), "$FREQ,%.3f,CH%u\r\n", (double)E220_FREQ_MHZ, E220_CHANNEL);
        radio.transmit(reinterpret_cast<uint8_t*>(freq_msg), (size_t)n);
    }

    fc.init();

    // --- BME680
    bool bme_ok = bme.init();
    if (!bme_ok) {
        printf("[WARN] BME680 not found — continuing without sensor\n");
        neopixel_set(100, 0, 0); // red = sensor error
        led_blink(5);
    } else {
        printf("[OK] BME680 initialised\n");
        neopixel_set(0, 100, 0); // green = all good
        led_blink(2);
    }

    printf("[OK] System ready. Starting 1Hz loop.\n");

    // --- Watchdog: enable after all init — radio.init() alone can take >2s
    watchdog_enable(WATCHDOG_MS, true /*pause on debug*/);

    // ── Main loop (1 Hz) ───────────────────────────────────────────────────
    uint32_t loop_count = 0;
    while (true) {
        absolute_time_t loop_start = get_absolute_time();
        watchdog_update();
        ++loop_count;

        // 1. Toggle LED to show life
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

        // 2. Drain FC UART; fc.update() is non-blocking
        for (int i = 0; i < 100; i++) fc.update();

        // 3. Read BME680 (blocks ~200ms in forced mode)
        BME680Data env{};
        if (bme_ok) {
            if (!bme.read(env)) {
                printf("[WARN] BME680 read failed\n");
            }
        }

        watchdog_update(); // pet after the blocking sensor read

        // 4. Build telemetry string
        char packet[160];
        uint32_t ts = to_ms_since_boot(get_absolute_time());
        int plen = build_packet(packet, sizeof(packet), ts, env, fc.data(), fc.bytes_received(), fc.stx_v1_count(), fc.stx_v2_count());

        // 6. Print to USB (debug)
        printf("%s", packet);

        // 6a. Log BME680 values to FC DataFlash via MAVLink NAMED_VALUE_FLOAT
        if (env.valid) {
            fc.send_named_float(ts, "TEMP",  env.temperature + TEMP_OFFSET_C);
            fc.send_named_float(ts, "PRESS", env.pressure);
            fc.send_named_float(ts, "HUMID", env.humidity);
            fc.send_named_float(ts, "GAS",   env.gas_resistance);
        }

        // 7. Re-broadcast frequency packet every 10 loops so the ground station
        //    can pick it up even if it connects after boot
        if (loop_count % 10 == 1) {
            char freq_msg[48];
            int fn = snprintf(freq_msg, sizeof(freq_msg),
                              "$FREQ,%.3f,CH%u\r\n",
                              (double)E220_FREQ_MHZ, E220_CHANNEL);
            radio.transmit(reinterpret_cast<uint8_t*>(freq_msg), (size_t)fn);
            watchdog_update();
        }

        // 9. Transmit telemetry via E220 and update NeoPixel status
        bool radio_ok = true;
        if (plen > 0) {
            radio_ok = radio.transmit(reinterpret_cast<uint8_t*>(packet), (size_t)plen);
            if (!radio_ok) {
                printf("[WARN] E220 transmit timeout (AUX not ready)\n");
            }
        }

        if (!radio_ok) {
            neopixel_set(255, 40, 0); // orange = transmit fail
        } else if (!env.valid) {
            neopixel_set(255, 255, 0); // yellow = sensor invalid, radio OK
        } else {
            neopixel_set(0, 255, 0);   // green = all good
        }

        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);

        // 10. Pace loop to ~1 Hz, continuing to drain FC data while waiting
        int64_t elapsed = absolute_time_diff_us(loop_start, get_absolute_time());
        int64_t remaining_us = (int64_t)(LOOP_PERIOD_MS * 1000LL) - elapsed;
        while (remaining_us > 0) {
            watchdog_update();
            fc.update();
            sleep_us(500);
            remaining_us -= 600;
        }
    }
}
