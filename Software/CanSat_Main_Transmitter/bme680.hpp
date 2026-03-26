#pragma once
#include "hardware/i2c.h"
#include <cstdint>

// I2C address: SDO→GND = 0x76, SDO→VCC = 0x77
static constexpr uint8_t BME680_ADDR = 0x76;

struct BME680Data {
    float temperature;   // °C
    float pressure;      // hPa
    float humidity;      // %RH
    float gas_resistance; // Ohms
    bool valid;
};

class BME680 {
public:
    explicit BME680(i2c_inst_t* i2c, uint8_t addr = BME680_ADDR);

    // Returns false if chip not found or init fails
    bool init();

    // Trigger a forced-mode measurement and block until ready (~200ms)
    bool read(BME680Data& out);

private:
    i2c_inst_t* i2c_;
    uint8_t     addr_;

    // Calibration coefficients
    uint16_t par_T1;
    int16_t  par_T2, par_T3_i;
    uint16_t par_P1;
    int16_t  par_P2, par_P4, par_P5, par_P8, par_P9;
    int8_t   par_P3, par_P6, par_P7;
    uint16_t par_P10;
    uint16_t par_H1, par_H2;
    int8_t   par_H3, par_H4, par_H5;
    uint8_t  par_H6;
    int8_t   par_H7;
    int8_t   par_G1;
    int16_t  par_G2;
    int8_t   par_G3;
    uint8_t  res_heat_range;
    int8_t   res_heat_val;
    int8_t   range_sw_err;

    // Ambient temp used for gas heater target calc
    float t_fine = 0.0f;

    void     write_reg(uint8_t reg, uint8_t val);
    uint8_t  read_reg(uint8_t reg);
    void     read_regs(uint8_t reg, uint8_t* buf, size_t len);
    bool     load_calibration();
    void     set_gas_heater(uint16_t target_temp_c, uint16_t duration_ms);
    uint8_t  calc_heater_res(uint16_t temp);
    uint8_t  calc_heater_dur(uint16_t dur_ms);

    float    comp_temperature(uint32_t adc_T);
    float    comp_pressure(uint32_t adc_P);
    float    comp_humidity(uint16_t adc_H);
    float    comp_gas(uint16_t adc_G, uint8_t gas_range);
};
