/*
 * This file is part of the c't-Bot ATmega framework.
 * Copyright (c) 2018 Timo Sandmann
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file    encoder.h
 * @brief   c't-Bot wheel encoder driver
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#ifndef SRC_ENCODER_H_
#define SRC_ENCODER_H_

#include "timer.h"

#include <cstdint>


namespace ctbot {

class Encoder {
    friend class DigitalSensors;

protected:
    struct enc_t {
        uint8_t ticks;
        uint8_t timer;
    } __attribute__((packed));

    static constexpr float AVG_FILTER_PARAM { 0.25f };
    static constexpr float WHEEL_PERIMETER { 178.1283 }; // mm
    static constexpr uint8_t ENCODER_MARKS { 60 };

    int16_t edges_;
    uint8_t last_idx_;
    float speed_, speed_avg_;
    bool direction_; // true: forward, false: backwards

    static void init_interrupts();

    const enc_t* const p_enc_data_;
    const volatile uint8_t* const p_enc_idx_;
    uint32_t last_update_;
    int8_t count_;

public:
    static constexpr uint8_t DATA_ARRAY_SIZE { 8 };

    Encoder(const enc_t* const p_data, const volatile uint8_t* const p_idx, volatile uint8_t* p_ddr, volatile uint8_t* p_port, uint8_t pin);

    void update();

    auto get() const { return edges_; }
    auto get_speed() const { return speed_avg_; }

    void set_direction(bool dir) {
        direction_ = dir;
    }

    template <uint8_t VECT_NUM, uint8_t ARRAY_SIZE>
    static inline __attribute__((always_inline)) void isr(enc_t* p_data, volatile uint8_t* p_idx, const uint8_t timer) {
        const auto now(Timer::get_tickcount<uint8_t, true>());

        uint8_t idx(*p_idx);
        ++idx;
        idx %= ARRAY_SIZE;
        *p_idx = idx;
        p_data[idx].timer = timer;
        p_data[idx].ticks = now;
    }
};

} /* namespace ctbot */

#endif /* SRC_ENCODER_H_ */
