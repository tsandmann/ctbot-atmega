/*
 * This file is part of a c++ utility library for platforms where no
 * full c++ standard library is available.
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
 * @file    serial_iostream_avr.cpp
 * @brief   Wrapper for std::cout, std::cerr and std::cin to ATmega serial port driver
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#include "serial_port.h"

#include <cstdio>
#include <iostream>
#include <serstream>


std::ohserialstream __cout { *avr::Uart0::get_impl() };
std::ihserialstream __cin { *avr::Uart0::get_impl() };

namespace std {
ostream& cout { __cout };
ostream& cerr { __cout };
istream& cin { __cin };
}

namespace avr {

/**
 * @brief Wrapper for std::cout, std::cerr and std::cin to ATmega serial port driver
 * @note Standard input/output streams are mapped to UART0 of ATmega
 */
class StdioWrapper {
public:
    /**
     * @brief Construct a new StdioWrapper object
     */
    StdioWrapper() {
        stdout_buf.put = _putchar;
        stdout_buf.get = _getchar;
        stdout_buf.flags = _FDEV_SETUP_RW;
        stdout_buf.udata = 0;
        stdout = &stdout_buf;
        stderr = &stdout_buf;
        stdin = &stdout_buf;
    }

private:
    static FILE stdout_buf;

    /**
     * @brief Output a single character to the serial port
     * @param[in] c: Character to output
     * @return 0 on success, 1 on error
     */
    static int _putchar(char c, FILE*);

    /**
     * @brief Take a character from the serial port
     * @return The character or -1 on a read error
     * @note Blocks until a character is available
     */
    static int _getchar(FILE*);
};


FILE StdioWrapper::stdout_buf;

int StdioWrapper::_putchar(char c, FILE*) {
    const SerialPort& uart { *Uart0::get_impl() };
    return uart.write(c) == 1 ? 0 : 1;
}

int StdioWrapper::_getchar(FILE*) {
    SerialPort& uart { *Uart0::get_impl() };
    while (! uart.available()) { /* wait */ }
    return uart.read();
}


static StdioWrapper stdio_wrapper; /**< Global StdioWrapper instance */

} /* namespace avr */
