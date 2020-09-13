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
 * @file    rc5_int.cpp
 * @brief   Interrupt driven RC5 decoder
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#include "rc5_int.h"

#include <utils_avr.h>
#include <rc5.h>
#include <iostream>
#include <type_traits>
#include <avr/io.h>


namespace ctbot {
using namespace avr;

std::remove_all_extents<decltype(Rc5::input_data_)>::type Rc5::input_data_[Rc5::DATA_ARRAY_SIZE];
decltype(Rc5::input_idx_) Rc5::input_idx_ { 0 };

Rc5::Rc5(volatile uint8_t* p_ddr, volatile uint8_t* p_port, volatile uint8_t* p_mask_reg, const uint8_t pin, const uint8_t pci)
    : last_idx_ { 0 }, rc5_addr_ { 0 }, rc5_cmd_ { 0 }, rc5_toggle_ { false }, p_impl_ { new RC5() } {
    CBI(p_ddr, pin); // set input
    SBI(p_port, pin); // enable pullup

    SBI(p_mask_reg, pin); // Pin Change Enable Mask
    SBI<uint8_t>(CtBotConfig::PCICR_, pci); // Pin Change Interrupt Enable
    SBI<uint8_t>(CtBotConfig::PCIFR_, pci); // clear int flag

    reset();
}

Rc5::~Rc5() {
    delete p_impl_;
}

void Rc5::reset() {
    last_idx_ = input_idx_;
    if (p_impl_) {
        p_impl_->reset();
    }
}

bool Rc5::update() {
    if (!p_impl_) {
        return false;
    }

    const uint8_t idx { input_idx_ };
    int8_t diff_rc5 { static_cast<int8_t>(idx - last_idx_) };
    if (diff_rc5 < 0) {
        diff_rc5 = static_cast<int8_t>(diff_rc5 + DATA_ARRAY_SIZE);
    }

    uint32_t now { Timer::get_tickcount<uint32_t>() };

    bool found { false };
    if (diff_rc5) {
        // std::cout << "idx=" << static_cast<uint16_t>(idx) << "\tlast_idx_=" << static_cast<uint16_t>(last_idx_) << "\tdiff_rc5=" <<
        // static_cast<uint16_t>(diff_rc5) << "\n";

        for (auto i { last_idx_ }; i != idx; i = static_cast<uint8_t>((i + 1) % DATA_ARRAY_SIZE)) {
            uint32_t ticks { input_data_[i].ticks };
            if (ticks <= (now & 0xffffUL)) {
                ticks = (now & 0xffff0000UL) | ticks;
            } else {
                ticks = ((now - 65536UL) & 0xffff0000UL) | ticks;
            }

            // std::cout << "i=" << static_cast<uint16_t>(i) << "\tticks=" << ticks;

            const uint32_t i_time { Timer::ticks_to_us(ticks, input_data_[i].timer) };
            const auto diff_time { i_time - last_time_ };
            last_time_ = i_time;
            // std::cout << "\ti_time=" << i_time << "\tdiff_time=" << diff_time << "\tvalue=" << input_data[i].value << "\n";
            // std::cout << "diff_time=" << diff_time << "\tvalue=" << input_data[i].value << "\n";

            if (p_impl_->read(rc5_toggle_, rc5_addr_, rc5_cmd_, input_data_[i].value, diff_time)) {
                found = true;
                // std::cout << "addr=" << static_cast<uint16_t>(rc5_addr_) << "\tcmd=" << static_cast<uint16_t>(rc5_cmd_) << "\ttoggle=" << rc5_toggle_ <<
                // "\n";
            }
        }
        last_idx_ = idx;
    }

    return found;
}

} /* namespace ctbot */
