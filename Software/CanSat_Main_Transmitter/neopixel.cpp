#include "neopixel.hpp"
#include "hardware/pio.h"
#include "ws2812.pio.h"

static PIO  np_pio = pio0;
static uint np_sm  = 0;

void neopixel_init() {
    uint offset = pio_add_program(np_pio, &ws2812_program);
    ws2812_program_init(np_pio, np_sm, offset, NEOPIXEL_PIN);
    neopixel_off();
}

void neopixel_set(uint8_t r, uint8_t g, uint8_t b) {
    // WS2812 expects GRB order, MSB first, left-justified in the 32-bit word
    uint32_t grb = ((uint32_t)g << 24) | ((uint32_t)r << 16) | ((uint32_t)b << 8);
    pio_sm_put_blocking(np_pio, np_sm, grb);
}
