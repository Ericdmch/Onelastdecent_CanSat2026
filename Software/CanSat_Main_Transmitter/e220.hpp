#pragma once
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include <cstdint>
#include <cstddef>

// Pin assignments
static constexpr uint E220_M0_PIN  = 2;
static constexpr uint E220_M1_PIN  = 3;
static constexpr uint E220_AUX_PIN = 6;

// E220 UART  (uart0 is a reinterpret_cast macro — cannot be constexpr)
#define E220_UART uart0
static constexpr uint E220_TX_PIN  = 0;
static constexpr uint         E220_RX_PIN  = 1;
static constexpr uint         E220_BAUD    = 9600; // default factory baud

class E220 {
public:
    void init();

    // Returns true if AUX is HIGH (module idle and ready to transmit)
    bool ready() const;

    // Transmit raw bytes; waits up to timeout_ms for AUX HIGH before sending
    // Returns false if timed out waiting for ready
    bool transmit(const uint8_t* data, size_t len, uint32_t timeout_ms = 500);

    // Convenience: transmit a null-terminated string
    bool transmit_str(const char* s, uint32_t timeout_ms = 500);
};
