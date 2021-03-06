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
 * @file    display.h
 * @brief   LC display driver for devices with Hitachi HD44780
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#ifndef SRC_DISPLAY_H_
#define SRC_DISPLAY_H_

#include "ctbot_config.h"
#include "shift_reg.h"

#include <utils_avr.h>
#include <cstdint>
#include <avr/io.h>


namespace ctbot {
using namespace avr;

/**
 * @brief LC Display driver implementation for devices with Hitachi HD44780
 */
class Display {
protected:
    static constexpr uint8_t LINE_LENGTH = 20; /**< Size of display (length of one line) */
    static constexpr uint8_t CMD_CLEAR = 0x1; /**< Command to clear display */
    static constexpr uint8_t CMD_CURSOR_HOME = 0x2; /**< Command to set cursor to position */
    static constexpr uint8_t CMD_SET_MODE = 0xe; /**< Command to turn display on, set cursor on, disable cursor blink */

    using LcdShiftReg = ShiftReg<CtBotConfig::DISPLAY_SCK_PIN, CtBotConfig::DISPLAY_RCK_PIN>;
    LcdShiftReg shiftreg_;
    char buffer_[LINE_LENGTH + 1];

    /**
     * @brief Set the ENA line low
     */
    void reset_ena() const {
        CBI<uint8_t>(CtBotConfig::DISPLAY_REG::PORT, CtBotConfig::DISPLAY_ENA_PIN);
    }

    /**
     * @brief Send a command to the display processor
     * @param[in] cmd: Command to send
     */
    void send_cmd(const uint8_t cmd) const;

public:
    Display();

    /**
     * @brief Clear entire display, set cursor to home position
     */
    void clear() const {
        send_cmd(CMD_CLEAR);
    }

    /**
     * @brief Set the cursor to a position
     * @param[in] row: New row of cursor [1; 20]
     * @param[in] column: New column of cursor [1; 4]
     */
    void set_cursor(const uint8_t row, const uint8_t column) const;

    /**
     * @brief Write a char to the display at the current position
     * @param[in] data: Char to write
     * @param[in] reduced_wait: Set to true, if the wait time after the data is send should be reduced to 35 us
     */
    void print(const char data, bool reduced_wait = false) const;

    /**
     * @brief Write a C-string stored in flash memory to the display, starting at the current position
     * @param[in] str: Pointer to the C-string stored in flash
     * @return Number of written chars
     */
    uint8_t print(const avr::FlashStringHelper* str) const;

    /**
     * @brief Write a formatted C-string stored in program memory to the display, starting at the current position
     * @param[in] format: printf()-like format string stored in program memory
     * @param[in] ...: Variadic argument list as for printf()
     * @return Number of written chars
     */
    uint8_t printf(const avr::FlashStringHelper* format, ...);
};

} /* namespace ctbot */

#endif /* SRC_DISPLAY_H_ */
