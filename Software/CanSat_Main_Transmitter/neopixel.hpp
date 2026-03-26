#pragma once
#include "pico/stdlib.h"
#include <cstdint>

static constexpr uint NEOPIXEL_PIN = 7;

void neopixel_init();

// Set colour (RGB). Keep values ≤ 100 to limit current draw on battery.
void neopixel_set(uint8_t r, uint8_t g, uint8_t b);
inline void neopixel_off() { neopixel_set(0, 0, 0); }
