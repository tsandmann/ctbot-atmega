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
 * @file    comm_interface.cpp
 * @brief   Communication interface classes of c't-Bot atmega framework
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#include "comm_interface.h"
#include "ctbot.h"
#include "cmd_parser.h"
#include "serial_connection_avr.h"
#include "timer.h"
#include "scheduler.h"
#include "speed_control.h"
#include "servo.h"
#include "leds.h"
#include "display.h"
#include "sensors.h"

#include <utils_avr.h>
#include <cmath>
#include <iostream>


namespace ctbot {

CommInterface::CommInterface(SerialConnectionAVR& io_connection, bool enable_echo) : io_(io_connection), echo_(enable_echo), error_(0), p_input_(input_buffer_) {
    CtBot::get_instance().get_scheduler()->task_add("comm", TASK_PERIOD_MS, [] (void* p_data) { auto p_this(reinterpret_cast<CommInterface*>(p_data)); return p_this->run(); }, this);
}

int16_t CommInterface::debug_print(const avr::FlashStringHelper* str) const {
	auto ptr(reinterpret_cast<PGM_P>(str));
	size_t n { 0 };
	while (true) {
		const uint8_t c { pgm_read_byte(ptr++) };
		if (c == 0) {
			break;
		}
		if (io_.send(&c, 1)) {
			++n;
		} else {
			break;
		}
	}
	return n;
}

int16_t CommInterface::debug_print(const char c) const {
	return io_.send(&c, 1);
}

int16_t CommInterface::debug_print(const char* str) const {
	return io_.send(str, std::strlen(str));
}

int16_t CommInterface::debug_print(const std::string& str) const {
	return io_.send(str.c_str(), str.length());
}

/**
 * @note based on Arduino Print::printNumber()
 */
int16_t CommInterface::print_uint(const uint32_t v, const PrintBase base) const {
	if (base == PrintBase::NONE) {
		return debug_print(static_cast<const char>(v));
	}

	uint32_t value { v };
	char buf[8 * sizeof(uint32_t) + 1]; // Assumes 8-bit chars plus zero byte
	char* str = &buf[sizeof(buf) - 1];
	*str = '\0';

	const uint8_t base_value { static_cast<const uint8_t>(base) };
	do {
		const char c { static_cast<const char>(value % base_value) };
		value /= base_value;
		*--str = c < 10 ? c + '0' : c + 'A' - 10;
	} while (value);

	return debug_print(str);
}

int16_t CommInterface::print_int(const int32_t v, const PrintBase base) const {
	if (base == PrintBase::NONE) {
		return print_uint(static_cast<uint32_t>(v), PrintBase::NONE);
	}

	if (base == PrintBase::DEC) {
		if (v < 0) {
			const int16_t t { debug_print('-') };
			const uint32_t u { static_cast<uint32_t>(-v) };
			return print_uint(u, PrintBase::DEC) + t;
		}
		return print_uint(static_cast<uint32_t>(v), PrintBase::DEC);
	} else {
		return print_uint(static_cast<uint32_t>(v), base);
	}
}

/**
 * @note based on Arduino Print::printFloat()
 */
int16_t CommInterface::debug_print(const float v, const uint8_t digits) const {
	float number { v };
	uint8_t d { digits };
	size_t n { 0U };

	if (std::isnan(number)) {
		return debug_print("nan");
	}
	if (std::isinf(number)) {
		return debug_print("inf");
	}
	if (number > 4294967040.f) {
		return debug_print("ovf"); // constant determined empirically
	}
	if (number < -4294967040.f) {
		return debug_print("ovf"); // constant determined empirically
	}

	// Handle negative numbers
	if (number < 0.f) {
		n += debug_print('-');
		number = -number;
	}

	// Round correctly so that print(1.999, 2) prints as "2.00"
	float rounding { 0.5f };
	for (uint8_t i { 0 }; i < d; ++i) {
		rounding /= 10.f;
	}

	number += rounding;

	// Extract the integer part of the number and print it
	const uint32_t int_part { static_cast<uint32_t>(number) };
	float remainder { number - static_cast<float>(int_part) };
	n += debug_print(int_part, PrintBase::DEC);

	// Print the decimal point, but only if there are digits beyond
	if (d > 0) {
		n += debug_print('.');
	}

	// Extract digits from the remainder one at a time
	while (d-- > 0) {
		remainder *= 10.f;
		uint16_t toPrint { static_cast<uint16_t>(remainder) };
		n += debug_print(toPrint, PrintBase::DEC);
		remainder -= toPrint;
	}

	return n;
}

CommInterfaceCmdParser::CommInterfaceCmdParser(SerialConnectionAVR& io_connection, CmdParser& parser, bool enable_echo) :
    CommInterface(io_connection, enable_echo), cmd_parser_(parser) {
    cmd_parser_.set_echo(enable_echo);
}

void CommInterfaceCmdParser::set_echo(bool value) {
    echo_ = value;
    cmd_parser_.set_echo(value);
}

void CommInterfaceCmdParser::run() {
    // std::cout << "CommInterfaceCmdParser::run(): " << Timer::get_ms() << " ms\n";
    size_t n { 0 };
    while (io_.available()) {
        char c;
        io_.receive(&c, 1);

        if (p_input_ >= &input_buffer_[sizeof(input_buffer_)]) {
            /* no buffer space left */
            p_input_ = input_buffer_;
            n = 0;
            error_ = 1;
        }
        ++n;

        if (c == '\n' || c == '\r') {
            *p_input_ = c;
            if (io_.peek() == '\r' || io_.peek() == '\n') {
                char tmp;
                io_.receive(&tmp, 1);
            }
            *p_input_ = 0;
            if (echo_) {
                io_.send("\n", 1);
            }
            cmd_parser_.parse(input_buffer_, *this);
            p_input_ = input_buffer_;
            break;
        } else if (c == '\b') {
            /* backspace */
            if (p_input_ > input_buffer_) {
                --p_input_;
                if (echo_) {
                    char tmp[] { "\b \b" };
                    io_.send(tmp, sizeof(tmp) - 1);
                }
            }
        } else {
            *p_input_++ = c;
            if (echo_) {
                io_.send(&c, 1);
            }
        }
    }
}

} /* namespace ctbot */
