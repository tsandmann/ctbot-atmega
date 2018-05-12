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
 * @file    shift_reg.h
 * @brief   Shift register 74HC595 driver
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#ifndef SRC_SHIFT_REG_H_
#define SRC_SHIFT_REG_H_

#include <cstdint>


namespace ctbot {

/**
 * @brief Shift register 74HC595 driver
 * @tparam SCK_PIN: Pin number for shift register clock input
 * @tparam RCK_PIN: Pin number for storage register clock input
 */
template <uint8_t SCK_PIN, uint8_t RCK_PIN>
class ShiftReg {
public:
    ShiftReg();
    /**
     * @brief Shift out 8 bit of data
     * @param[in] data: The data to be shifted out
     */
    void out(uint8_t data) const {
        out(data, 0, 0);
    }

    /**
     * @brief Shift out 8 bit of data
     * @param[in] data: The data to be shifted out
     * @param[in] pins_to_set: A mask of pins to be set to high after the shift operation
     * @param[in] pins_to_clear: A mask of pins to be set to low after the shift operation
     */
    void out(uint8_t data, const uint8_t pins_to_set, const uint8_t pins_to_clear = 0) const;
};

} /* namespace ctbot */

#endif /* SRC_SHIFT_REG_H_ */
