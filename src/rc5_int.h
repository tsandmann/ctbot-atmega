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
 * @file    rc5_int.h
 * @brief   Interrupt driven RC5 decoder
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#ifndef SRC_RC5_INT_H_
#define SRC_RC5_INT_H_

#include "timer.h"

#include <cstdint>


class RC5;

namespace ctbot {

class Rc5 {
    friend class DigitalSensors;

protected:
    struct rc5_t {
        uint16_t ticks;
        uint8_t timer;
        bool value;
    } __attribute__((packed));

    uint8_t last_idx_;
    uint32_t last_time_;
    uint8_t rc5_addr_;
    uint8_t rc5_cmd_;
    bool rc5_toggle_;
    RC5* p_impl_;

    void reset();

public:
    static constexpr uint8_t DATA_ARRAY_SIZE { 16 };
    static rc5_t input_data_[];
    static volatile uint8_t input_idx_;


    Rc5(volatile uint8_t* p_ddr, volatile uint8_t* p_port, volatile uint8_t* p_mask_reg, uint8_t pin, const uint8_t pci);

    ~Rc5();

    bool update();

    auto get_addr() const { return rc5_addr_; }
    auto get_cmd() const { return rc5_cmd_; }
    auto get_toggle() const { return rc5_toggle_; }
    void reset_rc5() {
        rc5_addr_ = rc5_cmd_ = 0;
    }

    template <uint8_t VECT_NUM, uint8_t ARRAY_SIZE>
    static inline __attribute__((always_inline)) void isr(const bool value, rc5_t* p_data, volatile uint8_t* p_idx, const uint8_t timer) {
        const auto now(Timer::get_tickcount<uint16_t, true>());

        uint8_t idx { *p_idx };
        p_data[idx].timer = timer;
        p_data[idx].ticks = now;
        p_data[idx].value = value;
        ++idx;
        idx %= ARRAY_SIZE;
        *p_idx = idx;
    }
};

} /* namespace ctbot */

#endif /* SRC_RC5_INT_H_ */
