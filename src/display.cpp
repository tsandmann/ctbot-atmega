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
 * @file    display.cpp
 * @brief   LC display driver for devices with Hitachi HD44780
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#include "display.h"

#include <avr/pgmspace.h>
#include <util/delay.h>


namespace ctbot {
using namespace avr;

Display::Display() {
    CBI<uint8_t>(CtBotConfig::DISPLAY_REG::DDR, CtBotConfig::DISPLAY_READY_PIN); // set to input

    /* wait 15 ms for display to boot */
    _delay_ms(15);

    /* set register in 8 bit mode */
    for (uint8_t i(0U); i < 3; ++i) {
        shiftreg_.out(0x38, false); // send command to display
        reset_ena(); // clear serial lines
        _delay_ms(5); // wait for 5 ms
    }

    /* set display and cursor mode */
    send_cmd(CMD_SET_MODE);
    _delay_us(1520);
    clear();
}

void Display::send_cmd(const uint8_t cmd) const {
    shiftreg_.out(cmd, false); // send command to display
    reset_ena(); // clear serial lines

    if (cmd == CMD_CLEAR) {
        _delay_us(1600); // wait for 1600 us
    } else {
        _delay_us(47); // wait for 37 us
    }
}

void Display::set_cursor(const uint8_t row, const uint8_t column) const {
    const uint8_t c { static_cast<uint8_t>(column - 1) };
    if (c >= LINE_LENGTH) {
        return;
    }

    switch (row) {
    case 1:
        send_cmd(0x80 + c);
        break;

    case 2:
        send_cmd(0xc0 + c);
        break;

    case 3:
        send_cmd(0x94 + c);
        break;

    case 4:
        send_cmd(0xd4 + c);
        break;
    }
}

void Display::print(const char data, bool reduced_wait) const {
    shiftreg_.out(data, BV_8(CtBotConfig::DISPLAY_RS_PIN), 0); // send char to display and set RS_PIN to high
    reset_ena(); // clear serial lines

    if (reduced_wait) {
        _delay_us(35); // wait for 35 us
    } else {
        _delay_us(47); // wait for 47 us
    }
}

uint8_t Display::print(const avr::FlashStringHelper* str) const {
    uint8_t len { 0U };
    char tmp;
    auto ptr(reinterpret_cast<const char*>(str));
    /* read C-string from flash memory */
    while ((tmp = (char) pgm_read_byte(ptr++)) != 0 && len < LINE_LENGTH) {
        print(tmp, true); // send character to display, reduce wait time because of pgm_read_byte() per iteration
        ++len;
    }

    return len;
}

uint8_t Display::printf(const avr::FlashStringHelper* format, ...) {
    va_list	args;
    va_start(args, format);

    /* read C-string from flash memory and parse format */
    uint8_t len(vsnprintf_P(buffer_, sizeof(buffer_), reinterpret_cast<const char*>(format), args));
    va_end(args);

    /* truncate to line length */
    if (len > LINE_LENGTH) {
        len = LINE_LENGTH;
    }

    /* send characters to display */
    const char* ptr(buffer_);
    for (uint8_t i { 0U }; i < len; ++i) {
        print(*ptr++);
    }

    return len;
}

} /* namespace ctbot */
