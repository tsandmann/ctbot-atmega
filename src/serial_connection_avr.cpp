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
 * @file    serial_connection_avr.cpp
 * @brief   Abstraction layer for serial communication on AVR devices
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#include "serial_connection_avr.h"
#include "timer.h"

#include <cstring>
#include <cstddef>
#include <streambuf>
#include <sstream>


namespace ctbot {

struct streambuf_helper {
    template <class T>
    static const char* get_data(T& buf) {
        struct helper : public T {
            static auto& _get_data(T& buf) {
                auto &buffer(reinterpret_cast<helper&>(buf));
                return buffer.data;
            }
        };

        return helper::_get_data(buf).c_str();
    }
};

decltype(SerialConnectionAVR::wait_callback_) SerialConnectionAVR::wait_callback_(nullptr);

uint16_t SerialConnectionAVR::wait_for_data(const uint16_t size, const uint16_t timeout_ms) noexcept {
    uint16_t bytes_available { available() };
    if (bytes_available >= size) {
        return size;
    }

    const auto start(Timer::get_us());
    auto now(start);
    const auto timeout_us(static_cast<uint32_t>(timeout_ms) * 1000UL);
    auto running_us(now - start);

    while ((bytes_available < size) && ((! timeout_ms) || (running_us < timeout_us))) {
        if (wait_callback_) {
            wait_callback_(this);
        }
        bytes_available = available();
        now = Timer::get_us();
        running_us = now - start;
    }

    return std::min<uint16_t>(bytes_available, size);
}

std::size_t SerialConnectionAVR::available() const {
    return io_stream_.available();
}

std::size_t SerialConnectionAVR::receive(void* data, const std::size_t size) {
    if (! size) {
        return 0;
    }

    return io_stream_.read(static_cast<uint8_t*>(data), size);
}

std::size_t SerialConnectionAVR::receive(std::streambuf& buf, const std::size_t size) {
    if (! size) {
        return 0;
    }

    uint16_t i;
    for (i = 0U; i < size; ++i) {
        buf.sputc(io_stream_.read());
    }
    return i;
}

std::size_t SerialConnectionAVR::receive_until(void* data, const char delim, const std::size_t maxsize) {
    const auto size16(static_cast<uint16_t>(maxsize));
    uint16_t n { 0 };
    char* ptr(reinterpret_cast<char*>(data));
    do {
        const int c { io_stream_.read() };
        if (c >= 0) {
            *ptr = c;
            ++n;
        } else {
// FIXME: delay / yield?
            if (wait_callback_) {
                wait_callback_(this);
            }
        }
    } while (*ptr++ != delim && n < size16);

    return n;
}

std::size_t SerialConnectionAVR::receive_until(void* data, const std::string& delim, const std::size_t maxsize) {
    const auto size16(static_cast<uint16_t>(maxsize));
    uint16_t n { 0 };
    char* ptr(reinterpret_cast<char*>(data));
    do {
        const int c { io_stream_.read() };
        if (c >= 0) {
            *ptr = c;
            ++ptr;
            ++n;
        } else {
// FIXME: delay / yield?
            if (wait_callback_) {
                wait_callback_(this);
            }
        }
    } while (std::strncmp(reinterpret_cast<const char*>(data), delim.c_str(), n) && n < size16);

    return n;
}

std::size_t SerialConnectionAVR::receive_until(std::streambuf& buf, const char delim, const std::size_t maxsize) {
    const auto size16(static_cast<uint16_t>(maxsize));
    uint16_t n { 0 };
    char tmp;
    do {
        const int c { io_stream_.read() };
        if (c >= 0) {
            tmp = c;
            buf.sputc(tmp);
            ++n;
        } else {
            tmp = 0;
// FIXME: delay / yield?
            if (wait_callback_) {
                wait_callback_(this);
            }
        }
    } while (tmp != delim && n < size16);

    return n;
}

std::size_t SerialConnectionAVR::receive_until(std::streambuf& buf, const std::string& delim, const std::size_t maxsize) {
    auto& buffer(reinterpret_cast<std::stringbuf&>(buf));
    const auto size16(static_cast<uint16_t>(maxsize));

    uint16_t n { 0 };
    do {
        const int c { io_stream_.read() };
        if (c >= 0) {
            buf.sputc(c);
            ++n;
        } else {
// FIXME: delay / yield?
            if (wait_callback_) {
                wait_callback_(this);
            }
        }
    } while (std::strncmp(streambuf_helper::get_data(buffer), delim.c_str(), n) && n < size16);

    return n;
}

std::size_t SerialConnectionAVR::receive_async(void* data, const std::size_t size, const uint32_t timeout_ms) {
    auto ptr(reinterpret_cast<uint8_t*>(data));
    const auto timeout(static_cast<uint16_t>(timeout_ms));

    const auto avail(available());
    const auto to_read(std::min(avail, size));
    size_t done { 0 };
    if (to_read) {
        done = receive(ptr, to_read);
        ptr += done;
    }

    const auto n(wait_for_data(size - done, timeout));
    done += receive(ptr, n);

    return done;
}

std::size_t SerialConnectionAVR::receive_async(std::streambuf& buf, const std::size_t size, const uint32_t timeout_ms) {
    const auto timeout(static_cast<uint16_t>(timeout_ms));

    const auto avail(available());
    const auto to_read(std::min(avail, size));
    size_t done { 0 };
    if (to_read) {
        done = receive(buf, to_read);
    }

    const auto n(wait_for_data(size - done, timeout));
    done += receive(buf, n);

    return done;
}

std::size_t SerialConnectionAVR::send(const void* data, const std::size_t size) {
    return io_stream_.write(data, size);
}

std::size_t SerialConnectionAVR::send(std::streambuf& buf, const std::size_t size) {
    const auto size16(static_cast<uint16_t>(size));
    for (auto i(0U); i < size16; ++i) {
        io_stream_.write(static_cast<uint8_t>(buf.sbumpc()));
    }
    return size;
}

int SerialConnectionAVR::peek() const {
    return static_cast<int>(io_stream_.peek());
}

} /* namespace ctbot */
