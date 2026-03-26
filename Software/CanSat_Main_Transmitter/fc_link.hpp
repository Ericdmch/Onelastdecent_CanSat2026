#pragma once
#include "hardware/uart.h"
#include <cstdint>

// UART1 for flight controller (ArduPilot MAVLink telemetry)
// uart1 is a reinterpret_cast macro — cannot be constexpr
#define FC_UART uart1
static constexpr uint         FC_TX_PIN = 8;
static constexpr uint         FC_RX_PIN = 9;
static constexpr uint         FC_BAUD   = 57600; // ArduPilot default telem baud

// MAVLink v1 message IDs we care about
static constexpr uint8_t MAVLINK_MSG_GLOBAL_POSITION_INT = 33;
static constexpr uint8_t MAVLINK_MSG_ATTITUDE            = 30;

struct FCData {
    int32_t  lat;       // 1e7 degrees
    int32_t  lon;       // 1e7 degrees
    int32_t  alt_mm;    // mm above MSL
    int16_t  rel_alt;   // cm above home
    float    roll;      // radians
    float    pitch;     // radians
    float    yaw;       // radians
    bool     gps_valid;
    bool     att_valid;
};

class FCLink {
public:
    void init();

    // Call frequently in main loop; returns true when new data was parsed
    bool update();

    const FCData& data() const { return data_; }

private:
    FCData data_{};

    // MAVLink v1 framing state machine
    enum class State {
        IDLE, GOT_STX, GOT_LEN, GOT_SEQ, GOT_SYS, GOT_COMP,
        PAYLOAD, GOT_CKA
    };

    State   state_  = State::IDLE;
    uint8_t payload_len_  = 0;
    uint8_t msg_id_       = 0;
    uint8_t payload_idx_  = 0;
    uint8_t payload_[255] = {};
    uint8_t cka_          = 0;
    uint8_t ckb_          = 0;

    static constexpr uint8_t MAVLINK_STX = 0xFE;

    void reset_state();
    bool feed_byte(uint8_t b);
    void parse_global_pos();
    void parse_attitude();

    // MAVLink X.25 CRC (incremental)
    uint8_t crc_a_ = 0, crc_b_ = 0;
    void    crc_init();
    void    crc_update(uint8_t b);
};
