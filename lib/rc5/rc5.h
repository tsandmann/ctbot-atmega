/*
 *  RC5 Arduino Library
 *  Guy Carpenter, Clearwater Software - 2013
 *
 *  Licensed under the BSD2 license, see LICENSE for details.
 *
 *  All text above must be included in any redistribution.
 */

/**
 * @file    rc5.h
 * @brief   RC5 decoder library
 * @author  Guy Carpenter
 * @author  Timo Sandmann
 * @date    15.04.2018
 * @note    Based on <http://clearwater.com.au/code/rc5> and <https://github.com/guyc/RC5>,
 *          adapted for use with the c't-Bot ATmega framework and ported to C++14
 */

#ifndef RC5_H_
#define RC5_H_

#include <cstdint>


class RC5 {
protected:
    static constexpr uint32_t MIN_SHORT {  444U }; // us
    static constexpr uint32_t MAX_SHORT { 1333U }; // us
    static constexpr uint32_t MIN_LONG  { 1334U }; // us
    static constexpr uint32_t MAX_LONG  { 2222U }; // us

    /*
     * Step by two, because it makes it possible to use the values as bit-shift counters
     * when making state-machine transitions. States are encoded as 2 bits, so we step by 2.
     */
    static constexpr uint8_t EVENT_SHORTSPACE   { 0U };
    static constexpr uint8_t EVENT_SHORTPULSE   { 2U };
    static constexpr uint8_t EVENT_LONGSPACE    { 4U };
    static constexpr uint8_t EVENT_LONGPULSE    { 6U };
    static constexpr uint8_t STATE_START1       { 0U };
    static constexpr uint8_t STATE_MID1         { 1U };
    static constexpr uint8_t STATE_MID0         { 2U };
    static constexpr uint8_t STATE_START0       { 3U };

    /*
     * Definitions for parsing the bitstream into discrete parts.
     * 14 bits are parsed as: [S1][S2][T][A A A A A][C C C C C C]
     * Bits are transmitted MSbit first.
     */
    static constexpr uint16_t S2_MASK       { 0x1000 }; // 1 bit
    static constexpr uint8_t  S2_SHIFT      { 12U };
    static constexpr uint16_t TOGGLE_MASK   { 0x800 }; // 1 bit
    static constexpr uint8_t  TOGGLE_SHIFT  { 11U };
    static constexpr uint16_t ADDRESS_MASK  { 0x7C0 }; // 5 bit
    static constexpr uint8_t  ADDRESS_SHIFT {  6U };
    static constexpr uint16_t COMMAND_MASK  { 0x3F }; // low 6 bit
    static constexpr uint8_t  COMMAND_SHIFT {  0U };

    /**
     * @brief Table of transitions, indexed by the current state
     *
     * Each byte in the table represents a set of 4 possible next states, packed as 4 x 2-bit values:
     * 8 bits DDCCBBAA, where AA are the low two bits, and AA = short space transition, BB = short pulse transition,
     * CC = long space transition, DD = long pulse transition
     * If a transition does not change the state, an error has occured and the state machine should reset.
     *
     * The transition table is:
     * 00 00 00 01  from state 0: short space->1
     * 10 01 00 01  from state 1: short pulse->0, long pulse->2
     * 10 01 10 11  from state 2: short space->3, long space->1
     * 11 11 10 11  from state 3: short pulse->2
     */
    static const uint8_t trans_[];

    uint8_t state_;
    uint16_t bits_;
    uint16_t command_;

    void decode_event(const uint8_t event);
    void decode_pulse(const uint8_t signal, const uint32_t period);
    bool read(uint16_t& message, const bool value, const uint32_t elapsed);

public:
    RC5();
    bool read(bool& toggle, uint8_t& addr, uint8_t& cmd, const bool value, const uint32_t elapsed);
    void reset();
};

#endif /* RC5_H_ */
