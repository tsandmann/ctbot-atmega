/*
 *  RC5 Arduino Library
 *  Guy Carpenter, Clearwater Software - 2013
 *
 *  Licensed under the BSD2 license, see LICENSE for details.
 *
 *  All text above must be included in any redistribution.
 */

/**
 * @file    rc5.cpp
 * @brief   RC5 decoder library
 * @author  Guy Carpenter
 * @author  Timo Sandmann
 * @date    15.04.2018
 * @note    Based on <http://clearwater.com.au/code/rc5> and <https://github.com/guyc/RC5>,
 *          adapted for use with the c't-Bot ATmega framework and ported to C++14
 */

#include "rc5.h"


const uint8_t RC5::trans_[] = { 0x01, 0x91, 0x9B, 0xFB };

RC5::RC5() : state_ { 0 }, bits_ { 0 }, command_ { 0 } {
    reset();
}

void RC5::reset() {
    state_ = STATE_MID1;
    bits_ = 1; // emit a 1 at start
    command_ = 1;
}

void RC5::decode_pulse(const uint8_t signal, const uint32_t period) {
    if (period >= MIN_SHORT && period <= MAX_SHORT) {
        decode_event(signal ? EVENT_SHORTPULSE : EVENT_SHORTSPACE);
    } else if (period >= MIN_LONG && period <= MAX_LONG) {
        decode_event(signal ? EVENT_LONGPULSE : EVENT_LONGSPACE);
    } else {
        /* time period out of range */
        reset();
    }
}

void RC5::decode_event(const uint8_t event) {
    /* find next state, 2 bits */
    const uint8_t newState { static_cast<uint8_t>((trans_[state_] >> event) & 0x3) };
    if (newState == state_) {
        /* no state change indicates error, -> reset */
        reset();
    } else {
        state_ = newState;
        if (newState == STATE_MID0) {
            /* always emit 0 when entering mid0 state */
            command_ = (command_ << 1) + 0;
            bits_++;
        } else if (newState == STATE_MID1) {
            /* always emit 1 when entering mid1 state */
            command_ = (command_ << 1) + 1;
            bits_++;
        }
    }
}

bool RC5::read(uint16_t& message, const bool value, const uint32_t elapsed) {
    decode_pulse(value, elapsed);

    if (bits_ == 14) {
        message = command_;
        command_ = 0;
        bits_ = 0;
        return true;
    }

    return false;
}

bool RC5::read(bool& toggle, uint8_t& addr, uint8_t& cmd, const bool value, const uint32_t elapsed) {
    uint16_t msg;
    if (!read(msg, value, elapsed)) {
        return false;
    }

    toggle = (msg & TOGGLE_MASK) >> TOGGLE_SHIFT;
    addr = static_cast<uint8_t>((msg & ADDRESS_MASK) >> ADDRESS_SHIFT);

    // Support for extended RC5: to get extended command, invert S2 and shift into command's 7th bit
    const uint8_t extended { static_cast<uint8_t>((~msg & S2_MASK) >> (S2_SHIFT - 7)) };
    cmd = static_cast<uint8_t>(((msg & COMMAND_MASK) >> COMMAND_SHIFT) | extended);

    reset();

    return true;
}
