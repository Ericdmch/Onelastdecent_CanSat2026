#include "bme680.hpp"
#include "pico/stdlib.h"
#include <cmath>
#include <cstring>

// Register map (subset)
namespace Reg {
    static constexpr uint8_t CHIP_ID      = 0xD0;
    static constexpr uint8_t RESET        = 0xE0;
    static constexpr uint8_t CTRL_HUM     = 0x72;
    static constexpr uint8_t CTRL_MEAS    = 0x74;
    static constexpr uint8_t CONFIG       = 0x75;
    static constexpr uint8_t CTRL_GAS_1   = 0x71;
    static constexpr uint8_t CTRL_GAS_0   = 0x70;
    static constexpr uint8_t GAS_WAIT_0   = 0x64;
    static constexpr uint8_t RES_HEAT_0   = 0x5A;
    static constexpr uint8_t MEAS_STATUS  = 0x1D;
    static constexpr uint8_t PRESS_MSB    = 0x1F;
    // Calibration base addresses
    static constexpr uint8_t CALIB_T1_LSB = 0xE9;
    static constexpr uint8_t CALIB_T2_LSB = 0x8A;
    static constexpr uint8_t CALIB_P1_LSB = 0x8E;
    static constexpr uint8_t CALIB_H2_MSB = 0xE1;
    static constexpr uint8_t CALIB_G      = 0xED;
}

static constexpr uint8_t BME680_CHIP_ID = 0x61;

BME680::BME680(i2c_inst_t* i2c, uint8_t addr)
    : i2c_(i2c), addr_(addr) {}

void BME680::write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    i2c_write_blocking(i2c_, addr_, buf, 2, false);
}

uint8_t BME680::read_reg(uint8_t reg) {
    uint8_t val = 0;
    i2c_write_blocking(i2c_, addr_, &reg, 1, true);
    i2c_read_blocking(i2c_, addr_, &val, 1, false);
    return val;
}

void BME680::read_regs(uint8_t reg, uint8_t* buf, size_t len) {
    i2c_write_blocking(i2c_, addr_, &reg, 1, true);
    i2c_read_blocking(i2c_, addr_, buf, len, false);
}

bool BME680::init() {
    // Soft reset
    write_reg(Reg::RESET, 0xB6);
    sleep_ms(10);

    if (read_reg(Reg::CHIP_ID) != BME680_CHIP_ID) return false;

    if (!load_calibration()) return false;

    // Humidity oversampling x1
    write_reg(Reg::CTRL_HUM, 0x01);
    // T×2, P×4, forced mode (set later per measurement)
    // IIR filter coefficient 3
    write_reg(Reg::CONFIG, (0x02 << 2));
    // Gas heater: 300°C for 150ms on heater profile 0
    set_gas_heater(300, 150);
    // Enable gas and select heater profile 0
    write_reg(Reg::CTRL_GAS_1, (1 << 4) | 0x00);

    return true;
}

bool BME680::load_calibration() {
    uint8_t coeff1[23], coeff2[14], coeff3[5];
    read_regs(Reg::CALIB_T2_LSB, coeff1, 23); // 0x8A..0xA0
    read_regs(0xA1, &coeff1[23], 1);           // 0xA1 = H1/H2 share
    read_regs(Reg::CALIB_H2_MSB, coeff2, 14); // 0xE1..0xEE
    read_regs(0xED, coeff3, 5);

    // Temperature
    par_T2 = (int16_t)((coeff1[1] << 8) | coeff1[0]);
    par_T3_i= (int8_t)coeff1[2];

    // Pressure
    par_P1  = (uint16_t)((coeff1[5] << 8) | coeff1[4]);
    par_P2  = (int16_t)((coeff1[7] << 8) | coeff1[6]);
    par_P3  = (int8_t)coeff1[8];
    par_P4  = (int16_t)((coeff1[11] << 8) | coeff1[10]);
    par_P5  = (int16_t)((coeff1[13] << 8) | coeff1[12]);
    par_P6  = (int8_t)coeff1[15];
    par_P7  = (int8_t)coeff1[14];
    par_P8  = (int16_t)((coeff1[19] << 8) | coeff1[18]);
    par_P9  = (int16_t)((coeff1[21] << 8) | coeff1[20]);
    par_P10 = (uint8_t)coeff1[22];

    // par_T1 lives at 0xE9/0xEA
    par_T1  = (uint16_t)((coeff2[9] << 8) | coeff2[8]);  // 0xEA / 0xE9

    // Humidity (coeff2 starts at 0xE1)
    par_H2  = (uint16_t)((coeff2[0] << 4) | (coeff2[1] >> 4));   // 0xE1 / 0xE2[7:4]
    par_H1  = (uint16_t)((coeff2[1] & 0x0F) | (coeff2[2] << 4)); // 0xE2[3:0] / 0xE3
    par_H3  = (int8_t)coeff2[3];   // 0xE4
    par_H4  = (int8_t)coeff2[4];   // 0xE5
    par_H5  = (int8_t)coeff2[5];   // 0xE6
    par_H6  = coeff2[6];            // 0xE7
    par_H7  = (int8_t)coeff2[7];   // 0xE8

    // Gas (0xEB=G2_LSB, 0xEC=G2_MSB, 0xED=G1, 0xEE=G3)
    par_G1  = (int8_t)coeff2[12];  // 0xED
    par_G2  = (int16_t)((coeff2[11] << 8) | coeff2[10]);  // 0xEC / 0xEB
    par_G3  = (int8_t)coeff2[13];  // 0xEE

    res_heat_range = (read_reg(0x02) & 0x30) >> 4;
    res_heat_val   = (int8_t)read_reg(0x00);
    range_sw_err   = (int8_t)((read_reg(0x04) & 0xF0) >> 4);

    return true;
}

uint8_t BME680::calc_heater_res(uint16_t temp) {
    float var1 = ((float)par_G1 / 16.0f) + 49.0f;
    float var2 = (((float)par_G2 / 32768.0f) * 0.0005f) + 0.00235f;
    float var3 = (float)par_G3 / 1024.0f;
    float var4 = var1 * (1.0f + (var2 * (float)temp));
    float var5 = var4 + (var3 * 25.0f); // assume 25°C ambient
    return (uint8_t)(3.4f * ((var5 * (4.0f / (4.0f + (float)res_heat_range)) *
                   (1.0f / (1.0f + ((float)res_heat_val * 0.002f)))) - 25.0f));
}

uint8_t BME680::calc_heater_dur(uint16_t dur_ms) {
    uint8_t factor = 0;
    while (dur_ms > 0x3F) { dur_ms /= 4; factor++; }
    return (uint8_t)(dur_ms | (factor << 6));
}

void BME680::set_gas_heater(uint16_t target_temp_c, uint16_t duration_ms) {
    write_reg(Reg::RES_HEAT_0,  calc_heater_res(target_temp_c));
    write_reg(Reg::GAS_WAIT_0,  calc_heater_dur(duration_ms));
}

float BME680::comp_temperature(uint32_t adc_T) {
    float var1 = ((float)adc_T / 16384.0f - (float)par_T1 / 1024.0f) * (float)par_T2;
    float var2 = (((float)adc_T / 131072.0f - (float)par_T1 / 8192.0f) *
                  ((float)adc_T / 131072.0f - (float)par_T1 / 8192.0f)) * ((float)par_T3_i * 16.0f);
    t_fine = var1 + var2;
    return t_fine / 5120.0f;
}

float BME680::comp_pressure(uint32_t adc_P) {
    float var1 = ((float)t_fine / 2.0f) - 64000.0f;
    float var2 = var1 * var1 * ((float)par_P6 / 131072.0f);
    var2 += var1 * (float)par_P5 * 2.0f;
    var2 = (var2 / 4.0f) + ((float)par_P4 * 65536.0f);
    var1 = (((float)par_P3 * var1 * var1 / 16384.0f) + ((float)par_P2 * var1)) / 524288.0f;
    var1 = (1.0f + var1 / 32768.0f) * (float)par_P1;
    if (var1 == 0.0f) return 0.0f;
    float press = 1048576.0f - (float)adc_P;
    press = ((press - (var2 / 4096.0f)) * 6250.0f) / var1;
    var1 = (float)par_P9 * press * press / 2147483648.0f;
    var2 = press * ((float)par_P8 / 32768.0f);
    float var3 = (press / 256.0f) * (press / 256.0f) * (press / 256.0f) * ((float)par_P10 / 131072.0f);
    press += (var1 + var2 + var3 + ((float)par_P7 * 128.0f)) / 16.0f;
    return press / 100.0f; // hPa
}

float BME680::comp_humidity(uint16_t adc_H) {
    float temp_comp = t_fine / 5120.0f;
    float var1 = (float)adc_H - (((float)par_H1 * 16.0f) + (((float)par_H3 / 2.0f) * temp_comp));
    float var2 = var1 * ((float)par_H2 / 262144.0f *
                 (1.0f + ((float)par_H4 / 67108864.0f * temp_comp) +
                  ((float)par_H5 / 67108864.0f * temp_comp * temp_comp)));
    float var3 = (float)par_H6 / 16384.0f;
    float var4 = (float)par_H7 / 2097152.0f;
    float hum  = var2 + (var3 + var4 * var2) * var2 * var2;
    if (hum > 100.0f) hum = 100.0f;
    if (hum < 0.0f)   hum = 0.0f;
    return hum;
}

float BME680::comp_gas(uint16_t adc_G, uint8_t gas_range) {
    static const uint32_t const_array1_int[16] = {
        2147483647UL, 2147483647UL, 2147483647UL, 2147483647UL,
        2147483647UL, 2126008810UL, 2147483647UL, 2130303777UL,
        2147483647UL, 2147483647UL, 2143188679UL, 2136746228UL,
        2147483647UL, 2126008810UL, 2147483647UL, 2147483647UL
    };
    static const uint32_t const_array2_int[16] = {
        4096000000UL, 2048000000UL, 1024000000UL, 512000000UL,
        255744255UL,  127110228UL,  64000000UL,   32258064UL,
        16016016UL,   8000000UL,    4000000UL,    2000000UL,
        1000000UL,    500000UL,     250000UL,      125000UL
    };
    int64_t var1 = (int64_t)((1340 + (5 * (int64_t)range_sw_err)) *
                   (int64_t)const_array1_int[gas_range]) >> 16;
    uint64_t var2 = (((int64_t)adc_G << 15) - 16777216LL) + var1;
    uint64_t var3 = (uint64_t)const_array2_int[gas_range] * (uint64_t)var1 >> 9;
    return (float)((var3 + (var2 >> 1)) / var2);
}

bool BME680::read(BME680Data& out) {
    out.valid = false;

    // Forced mode: T×2, P×4, mode=01
    write_reg(Reg::CTRL_MEAS, (0x02 << 5) | (0x05 << 2) | 0x01);

    // Wait for measurement complete (new_data_0 bit set)
    uint8_t status = 0;
    for (int i = 0; i < 50; i++) {
        sleep_ms(10);
        status = read_reg(Reg::MEAS_STATUS);
        if (status & 0x80) break;
    }
    if (!(status & 0x80)) return false;

    uint8_t raw[15];
    read_regs(Reg::PRESS_MSB, raw, 15);

    uint32_t adc_P = ((uint32_t)raw[0] << 12) | ((uint32_t)raw[1] << 4) | (raw[2] >> 4);
    uint32_t adc_T = ((uint32_t)raw[3] << 12) | ((uint32_t)raw[4] << 4) | (raw[5] >> 4);
    uint16_t adc_H = ((uint16_t)raw[6] << 8)  | raw[7];
    uint16_t adc_G = ((uint16_t)raw[11] << 2)  | (raw[12] >> 6);  // 0x2A / 0x2B
    uint8_t  gas_range = raw[12] & 0x0F;
    bool     gas_valid = (raw[12] & 0x20) && (raw[12] & 0x10);   // gas_valid=bit5, heat_stab=bit4

    out.temperature   = comp_temperature(adc_T);
    out.pressure      = comp_pressure(adc_P);
    out.humidity      = comp_humidity(adc_H);
    out.gas_resistance= gas_valid ? comp_gas(adc_G, gas_range) : -1.0f;
    out.valid         = true;
    return true;
}
