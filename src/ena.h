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
 * @file    ena.h
 * @brief   Abstraction layer for c't-Bot enable transistors
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#ifndef SRC_ENA_H_
#define SRC_ENA_H_

#include "ctbot_config.h"
#include "shift_reg.h"

#include <cstdint>


namespace ctbot {

/**
 * @brief Enum class for all enable signals
 */
enum class EnaTypes : uint8_t {
    NONE 		= 0,
    DISTANCE	= 1 << 0,
    WHEEL_ENC	= 1 << 1,
    TRANSPORT	= 1 << 2,
    BORDER		= 1 << 3,
    SHUTTER		= 1 << 4,
    LINE		= 1 << 5,
    EXTENSION_1	= 1 << 6,
    EXTENSION_2	= 1 << 7,
};

/**
 * @brief OR operator for EnaTypes
 * @param[in] lhs: Left hand side operand
 * @param[in] rhs: Right hand side operand
 * @return lhs OR rhs
 */
inline constexpr EnaTypes operator | (EnaTypes lhs, EnaTypes rhs) {
    return static_cast<EnaTypes>(static_cast<uint8_t>(lhs) | static_cast<uint8_t>(rhs));
}

/**
 * @brief AND operator for EnaTypes
 * @param[in] lhs: Left hand side operand
 * @param[in] rhs: Right hand side operand
 * @return lhs AND rhs
 */
inline constexpr EnaTypes operator & (EnaTypes lhs, EnaTypes rhs) {
    return static_cast<EnaTypes>(static_cast<uint8_t>(lhs) & static_cast<uint8_t>(rhs));
}

/**
 * @brief NOT operator for EnaTypes
 * @param[in] rhs: Right hand side operand
 * @return NOT rhs
 */
inline constexpr EnaTypes operator ~ (EnaTypes rhs) {
    return static_cast<EnaTypes>(~static_cast<uint8_t>(rhs));
}

/**
 * @brief Enable transistor driver
 */
class Ena {
protected:
    using EnaShiftReg = ShiftReg<CtBotConfig::ENA_SCK_PIN, CtBotConfig::ENA_RCK_PIN>;
    EnaShiftReg shiftreg_;
    EnaTypes status_;

    /**
     * @brief Updates the shift register to switch the ENA transistors
     * @note Shitfs out NOT status_, because of the low-active transistors
     */
    void update() const {
        shiftreg_.out(~static_cast<uint8_t>(status_));
    }

public:
    Ena() : status_(EnaTypes::NONE) {
        update();
    }

    /**
     * @brief Enables one or more transistors given by the mask enable
     * @param[in] enable: Bitmask of ENA transistors to be enabled
     * @note Transistors not selected by the mask are uneffected
     */
    void on(EnaTypes enable) {
        status_ = status_ | enable;
        update();
    }

    /**
     * @brief Disables one or more transistors given by the mask disable
     * @param[in] disable: Bitmask of ENA transistors to be disabled
     * @note Transistors not selected by the mask are uneffected
     */
    void off(EnaTypes disable) {
        status_ = status_ & (~disable);
        update();
    }

    /**
     * @brief Enables the transistors given by the mask, disabled the other ones
     * @param[in] mask: Bitmask of ENA transistors to be set
     */
    void set(EnaTypes mask) {
        status_ = mask;
        update();
    }
};

} /* namespace ctbot */

#endif /* SRC_ENA_H_ */
