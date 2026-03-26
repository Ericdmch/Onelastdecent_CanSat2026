#include "fc_link.hpp"
#include "pico/stdlib.h"
#include <cstring>

// MAVLink CRC-16/MCRF4XX extra bytes (CRC_EXTRA) for messages we care about
// These are defined in common.xml
static constexpr uint8_t CRC_EXTRA_ATTITUDE            = 39;
static constexpr uint8_t CRC_EXTRA_GLOBAL_POSITION_INT = 104;

// Helper: read little-endian integers from payload
static inline int32_t read_i32(const uint8_t* p) {
    return (int32_t)(p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24));
}
static inline float read_f32(const uint8_t* p) {
    float v; memcpy(&v, p, 4); return v;
}
static inline int16_t read_i16(const uint8_t* p) {
    return (int16_t)(p[0] | (p[1] << 8));
}

void FCLink::init() {
    uart_init(FC_UART, FC_BAUD);
    gpio_set_function(FC_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(FC_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(FC_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(FC_UART, true);
}

void FCLink::crc_init() { crc_a_ = 0; crc_b_ = 0; }

void FCLink::crc_update(uint8_t b) {
    uint8_t tmp = b ^ crc_a_;
    tmp ^= (tmp << 4);
    crc_a_ = (crc_b_ ^ (tmp >> 3) ^ (tmp << 4));
    crc_b_ = tmp;
}

void FCLink::reset_state() {
    state_       = State::IDLE;
    payload_idx_ = 0;
    crc_init();
}

bool FCLink::feed_byte(uint8_t b) {
    switch (state_) {
    case State::IDLE:
        if (b == MAVLINK_STX) { reset_state(); state_ = State::GOT_STX; }
        break;

    case State::GOT_STX:
        payload_len_ = b; crc_update(b); state_ = State::GOT_LEN; break;

    case State::GOT_LEN:
        crc_update(b); state_ = State::GOT_SEQ; break; // seq

    case State::GOT_SEQ:
        crc_update(b); state_ = State::GOT_SYS; break; // sysid

    case State::GOT_SYS:
        crc_update(b); state_ = State::GOT_COMP; break; // compid

    case State::GOT_COMP:
        msg_id_ = b; crc_update(b);
        payload_idx_ = 0;
        state_ = (payload_len_ > 0) ? State::PAYLOAD : State::GOT_CKA;
        break;

    case State::PAYLOAD:
        payload_[payload_idx_++] = b;
        crc_update(b);
        if (payload_idx_ >= payload_len_) state_ = State::GOT_CKA;
        break;

    case State::GOT_CKA:
        cka_ = b; state_ = State::IDLE; // next byte handled below, not here
        // We need one more byte for ckb — add a special state
        // Re-implemented below in update() directly
        break;
    }
    return false;
}

bool FCLink::update() {
    if (!uart_is_readable(FC_UART)) return false;

    bool got_msg = false;

    // Drain all available bytes
    while (uart_is_readable(FC_UART)) {
        uint8_t b = uart_getc(FC_UART);

        // Inline state machine (simpler than the method above for two-byte CRC)
        switch (state_) {
        case State::IDLE:
            if (b == MAVLINK_STX) {
                crc_init();
                state_ = State::GOT_STX;
            }
            break;

        case State::GOT_STX:
            payload_len_ = b; crc_update(b); state_ = State::GOT_LEN; break;

        case State::GOT_LEN:
            crc_update(b); state_ = State::GOT_SEQ; break;

        case State::GOT_SEQ:
            crc_update(b); state_ = State::GOT_SYS; break;

        case State::GOT_SYS:
            crc_update(b); state_ = State::GOT_COMP; break;

        case State::GOT_COMP:
            msg_id_ = b; crc_update(b);
            payload_idx_ = 0;
            state_ = (payload_len_ > 0) ? State::PAYLOAD : State::GOT_CKA;
            break;

        case State::PAYLOAD:
            payload_[payload_idx_++] = b;
            crc_update(b);
            if (payload_idx_ >= payload_len_) state_ = State::GOT_CKA;
            break;

        case State::GOT_CKA:
            // b is CKA checksum byte
            cka_ = b;
            state_ = State::IDLE; // re-use IDLE slot to hold "need CKB"
            // Transition to a fake state — we'll treat the next byte directly
            state_ = (State)99; // sentinel for "waiting for ckb"
            break;

        default: // state == 99: waiting for CKB
        {
            // Apply CRC_EXTRA before checking
            uint8_t extra = 0;
            if      (msg_id_ == MAVLINK_MSG_GLOBAL_POSITION_INT) extra = CRC_EXTRA_GLOBAL_POSITION_INT;
            else if (msg_id_ == MAVLINK_MSG_ATTITUDE)            extra = CRC_EXTRA_ATTITUDE;
            else { state_ = State::IDLE; break; } // unknown, skip check

            crc_update(extra);
            // crc_a_/crc_b_ now hold the computed CRC
            if (cka_ == crc_a_ && b == crc_b_) {
                if      (msg_id_ == MAVLINK_MSG_GLOBAL_POSITION_INT) { parse_global_pos(); got_msg = true; }
                else if (msg_id_ == MAVLINK_MSG_ATTITUDE)             { parse_attitude();   got_msg = true; }
            }
            state_ = State::IDLE;
            break;
        }
        } // switch
    } // while

    return got_msg;
}

void FCLink::parse_global_pos() {
    // GLOBAL_POSITION_INT (msg 33) payload layout:
    // 0: time_boot_ms (u32), 4: lat (i32), 8: lon (i32),
    // 12: alt (i32 mm), 16: relative_alt (i32 mm), 20: vx, 24: vy, 28: vz (i16 each), 32: hdg (u16)
    if (payload_len_ < 28) return;
    data_.lat     = read_i32(payload_ + 4);
    data_.lon     = read_i32(payload_ + 8);
    data_.alt_mm  = read_i32(payload_ + 12);
    data_.rel_alt = (int16_t)(read_i32(payload_ + 16) / 10); // mm→cm
    data_.gps_valid = true;
}

void FCLink::parse_attitude() {
    // ATTITUDE (msg 30) payload layout:
    // 0: time_boot_ms (u32), 4: roll (f), 8: pitch (f), 12: yaw (f), 16,20,24: rollspeed,pitchspeed,yawspeed (f)
    if (payload_len_ < 16) return;
    data_.roll  = read_f32(payload_ + 4);
    data_.pitch = read_f32(payload_ + 8);
    data_.yaw   = read_f32(payload_ + 12);
    data_.att_valid = true;
}
