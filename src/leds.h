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
 * @file    leds.h
 * @brief   Abstraction layer for c't-Bot LED switching
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#ifndef SRC_LEDS_H_
#define SRC_LEDS_H_

#include "ctbot_config.h"
#include "shift_reg.h"

#include <cstdint>


namespace ctbot {

enum class LedTypes : uint8_t {
    NONE 	= 0,
    RIGHT	= 1 << 0,
    LEFT	= 1 << 1,
    RED		= 1 << 2,
    ORANGE	= 1 << 3,
    YELLOW	= 1 << 4,
    GREEN	= 1 << 5,
    BLUE	= 1 << 6,
    WHITE	= 1 << 7,
};

inline LedTypes operator | (LedTypes a, LedTypes b)	{
    return static_cast<LedTypes>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}

inline LedTypes operator & (LedTypes a, LedTypes b)	{
    return static_cast<LedTypes>(static_cast<uint8_t>(a) & static_cast<uint8_t>(b));
}

inline LedTypes operator ~ (LedTypes a)	{
    return static_cast<LedTypes>(~static_cast<uint8_t>(a));
}

class Leds {
protected:
    using LedShiftReg = ShiftReg<CtBotConfig::LED_SCK_PIN, CtBotConfig::LED_RCK_PIN>;
    LedShiftReg shiftreg_;
    LedTypes status_;

public:
    Leds() : status_(LedTypes::NONE) {}

    auto get() const {
        return status_;
    }

    void set(const LedTypes leds) {
        status_ = leds;
        shiftreg_.out(static_cast<uint8_t>(leds));
    }

    void on(const LedTypes leds) {
        set(status_ | leds);
    }

    void off(const LedTypes leds) {
        set(status_ & ~leds);
    }
};

} /* namespace ctbot */

#endif /* SRC_LEDS_H_ */
