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
 * @file    shift_reg.cpp
 * @brief   Shift register 74HC595 driver
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#include "shift_reg.h"
#include "ctbot_config.h"

#include <utils_avr.h>
#include <avr/io.h>


namespace ctbot {
using namespace avr;

template <uint8_t SCK_PIN, uint8_t RCK_PIN>
ShiftReg<SCK_PIN, RCK_PIN>::ShiftReg() {
    /* set pins to output mode */
    SBI<uint8_t>(CtBotConfig::SHIFT_REG::DDR, CtBotConfig::SHIFT_SDATA_PIN);
    SBI<uint8_t>(CtBotConfig::SHIFT_REG::DDR, RCK_PIN);
    if (SCK_PIN != CtBotConfig::SHIFT_SDATA_PIN && SCK_PIN != RCK_PIN) {
        SBI<uint8_t>(CtBotConfig::SHIFT_REG::DDR, SCK_PIN);
    }
}

template <uint8_t SCK_PIN, uint8_t RCK_PIN>
void ShiftReg<SCK_PIN, RCK_PIN>::out(uint8_t data, const uint8_t pins_to_set, const uint8_t pins_to_clear) const {
    CBI<uint8_t>(CtBotConfig::SHIFT_REG::PORT, RCK_PIN); // reset RCK
    for (uint8_t i { 0U }; i < 8; ++i) {
        CBI<uint8_t>(CtBotConfig::SHIFT_REG::PORT, SCK_PIN); // reset SCK
        /* put MSB of data on SER */
        const uint8_t tmp((data >> 7) & 1);
        if (tmp) {
            SBI<uint8_t>(CtBotConfig::SHIFT_REG::PORT, CtBotConfig::SHIFT_SDATA_PIN);
        } else {
            CBI<uint8_t>(CtBotConfig::SHIFT_REG::PORT, CtBotConfig::SHIFT_SDATA_PIN);
        }
        SBI<uint8_t>(CtBotConfig::SHIFT_REG::PORT, SCK_PIN); // latch to storage -> rising edge on SCK
        data = static_cast<uint8_t>(data << 1);
    }

    ExecuteAtomic<> x;
    x([pins_to_clear, pins_to_set]() {
        uint8_t tmp { *PTR_8(CtBotConfig::SHIFT_REG::PORT) };
        tmp = static_cast<uint8_t>(tmp & ~((1 << (SCK_PIN)) | (1 << (CtBotConfig::SHIFT_SDATA_PIN)) | pins_to_clear));
        tmp = tmp | BV_8(RCK_PIN) | pins_to_set; // latch to output -> rising edge on RCK
        *PTR_8(CtBotConfig::SHIFT_REG::PORT) = tmp;
    });
}

/* explicit instantiation for shift regs used by c't-Bot */
template class ShiftReg<1, 2>; // display
template class ShiftReg<3, 1>; // ENA
template class ShiftReg<4, 1>; // LEDs

} /* namespace ctbot */
