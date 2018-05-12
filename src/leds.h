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

/**
 * @brief Enum class for all LEDs
 *
 */
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

/**
 * @brief OR operator for LedTypes
 * @param[in] lhs: Left hand side operand
 * @param[in] rhs: Right hand side operand
 * @return lhs OR rhs
 */
inline LedTypes operator | (LedTypes lhs, LedTypes rhs)	{
    return static_cast<LedTypes>(static_cast<uint8_t>(lhs) | static_cast<uint8_t>(rhs));
}

/**
 * @brief AND operator for LedTypes
 * @param[in] lhs: Left hand side operand
 * @param[in] rhs: Right hand side operand
 * @return lhs AND rhs
 */
inline LedTypes operator & (LedTypes lhs, LedTypes rhs)	{
    return static_cast<LedTypes>(static_cast<uint8_t>(lhs) & static_cast<uint8_t>(rhs));
}

/**
 * @brief NOT operator for LedTypes
 * @param[in] rhs: Right hand side operand
 * @return NOT rhs
 */
inline LedTypes operator ~ (LedTypes rhs)	{
    return static_cast<LedTypes>(~static_cast<uint8_t>(rhs));
}

/**
 * @brief LED driver
 */
class Leds {
protected:
    using LedShiftReg = ShiftReg<CtBotConfig::LED_SCK_PIN, CtBotConfig::LED_RCK_PIN>;
    LedShiftReg shiftreg_;
    LedTypes status_;

public:
    /**
     * @brief Construct a new Leds object
     */
    Leds() : status_(LedTypes::NONE) {}

    /**
     * @return Current LED setting as bitmask
     */
    auto get() const {
        return status_;
    }

    /**
     * @brief Activate and deactivate LEDs as given by a bitmask
     * @param[in] leds: Bitmask for LEDs to set
     */
    void set(const LedTypes leds) {
        status_ = leds;
        shiftreg_.out(static_cast<uint8_t>(leds));
    }

    /**
     * @brief Activate additional LEDs as given by an enable bitmask
     * @param[in] leds: Bitmask for LEDs to activate (others are not affected)
     */
    void on(const LedTypes leds) {
        set(status_ | leds);
    }

    /**
     * @brief Deactivate LEDs as given by a disable bitmask
     * @param[in] leds: Bitmask for LEDs to deactivate (others are not affected)
     */
    void off(const LedTypes leds) {
        set(status_ & ~leds);
    }
};

} /* namespace ctbot */

#endif /* SRC_LEDS_H_ */
