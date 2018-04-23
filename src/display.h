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

class Display {
protected:
    static constexpr uint8_t LINE_LENGTH = 20; /**< size of display (length of one line) */
    static constexpr uint8_t CMD_CLEAR = 0x1; /**< clear display */
    static constexpr uint8_t CMD_CURSOR_HOME = 0x2; /**< set cursor to position */
    static constexpr uint8_t CMD_SET_MODE = 0xe; /**< turn display on, set cursor on, disable cursor blink */

    using LcdShiftReg = ShiftReg<CtBotConfig::DISPLAY_SCK_PIN, CtBotConfig::DISPLAY_RCK_PIN>;
    LcdShiftReg shiftreg_;
    char buffer_[LINE_LENGTH + 1];

    /**
     * @brief Sets the ENA line low
     */
    void reset_ena() const {
        CBI<uint8_t>(CtBotConfig::DISPLAY_REG::PORT, CtBotConfig::DISPLAY_ENA_PIN);
    }

    /**
     * @brief Sends a command to the display processor
     * @param[in] cmd command to send
     */
    void send_cmd(const uint8_t cmd) const;

public:
    Display();

    /**
     * @brief Clears entire display, sets cursor to home position
     */
    void clear() const {
        send_cmd(CMD_CLEAR);
    }

    /**
     * @brief Sets the cursor to a position
     * @param[in] row new row of cursor [1; 20]
     * @param[in] column new column of cursor [1; 4]
     */
    void set_cursor(const uint8_t row, const uint8_t column) const;

    /**
     * @brief Writes a char to the display at the current position
     * @param[in] data char to write
     * @param[in] reduced_wait set to true, if the wait time after the data is send, should be reduced to 35 us
     */
    void print(const char data, bool reduced_wait = false) const;

    /**
     * @brief Writes a C-string stored in flash memory to the display, starting at the current position
     * @param[in] str pointer to the C-string stored in flash
     * @return number of written chars
     */
    uint8_t print(const avr::FlashStringHelper* str) const;

    /**
     * @brief Writes a formatted C-string stored in flash memory to the display, starting at the current position
     * @param[in] *format printf()-like format string stored in flash
     * @param[in] ... variable argument list as for printf()
     * @return number of written chars
     */
    uint8_t printf(const avr::FlashStringHelper* format, ...);
};

} /* namespace ctbot */

#endif /* SRC_DISPLAY_H_ */
