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
 * @file    serial_connection_avr.h
 * @brief   Abstraction layer for serial communication on AVR devices
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#ifndef SERIAL_CONNECTION_AVR_H_
#define SERIAL_CONNECTION_AVR_H_

#include <serial_port.h>
#include <streambuf>


namespace ctbot {

// FIXME: maybe this class can be simplified a lot...
class SerialConnectionAVR {
protected:
    avr::SerialPort& io_stream_;

    static void (*wait_callback_)(const void*);

    uint16_t wait_for_data(const uint16_t size, const uint16_t timeout_ms);

public:
    SerialConnectionAVR(avr::SerialPort& io) : io_stream_(io) {}

    ~SerialConnectionAVR() = default;

    SerialConnectionAVR(const SerialConnectionAVR&) = delete;
    SerialConnectionAVR& operator=(const SerialConnectionAVR&) = delete;

    static void set_wait_callback(decltype(wait_callback_) callback) {
        wait_callback_ = callback;
    }

    std::size_t available() const;
    std::size_t receive(void* data, const std::size_t size);
    std::size_t receive(std::streambuf& buf, const std::size_t size);
    std::size_t receive_until(void* data, const char delim, const std::size_t maxsize);
    std::size_t receive_until(void* data, const std::string& delim, const std::size_t maxsize);
    std::size_t receive_until(std::streambuf& buf, const std::string& delim, const std::size_t maxsize);
    std::size_t receive_until(std::streambuf& buf, const char delim, const std::size_t maxsize);
    std::size_t receive_async(std::streambuf& buf, const std::size_t size, const uint32_t timeout_ms);
    std::size_t receive_async(void* data, const std::size_t size, const uint32_t timeout_ms);
    std::size_t send(const void* data, const std::size_t size);
    std::size_t send(std::streambuf& buf, const std::size_t size);
    int peek() const;
};

} /* namespace ctbot */

#endif /* SERIAL_CONNECTION_AVR_H_ */
