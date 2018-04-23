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
 * @file    serial_port.h
 * @brief   ATmega1284P serial port driver
 * @author  Timo Sandmann
 * @date    15.04.2018
 * @note    Based on Arduino implementation <https://github.com/arduino/ArduinoCore-avr>
 */

#ifndef SRC_SERIAL_PORT_H_
#define SRC_SERIAL_PORT_H_

#include <cstdint>
#include <cstddef>
#include <type_traits>
#include <avr/io.h>


namespace avr {

static constexpr uint16_t SERIAL_RX_BUFFER_SIZE { 128 };

template <size_t RX_BUF_SIZE>
class SerialPortBase {
protected:
    using rx_buf_idx_t = typename std::conditional<RX_BUF_SIZE <= 256, uint8_t, uint16_t>::type;
    rx_buf_idx_t rx_buf_head_;
    rx_buf_idx_t rx_buf_tail_;
    uint8_t rx_buffer_[RX_BUF_SIZE];

public:
    SerialPortBase() : rx_buf_head_ { 0 }, rx_buf_tail_ { 0 } {}
};


class SerialPort : public SerialPortBase<SERIAL_RX_BUFFER_SIZE> {
public:
    SerialPort() = default;
    ~SerialPort() = default;

    void begin(const uint32_t baud) const;

    void end();

    void flush() const;

    uint16_t available() const {
        return (static_cast<uint16_t>(sizeof(rx_buffer_) + rx_buf_head_ - rx_buf_tail_)) % sizeof(rx_buffer_);
    }

    int16_t peek() const {
        return rx_buf_head_ == rx_buf_tail_ ? -1 : static_cast<int16_t>(rx_buffer_[rx_buf_tail_]);
    }

    int16_t read();

    int16_t read(void* buffer, const size_t length);

    int16_t write(const uint8_t) const;

    int16_t write(const void* buffer, const size_t length) const;

    inline void isr();
};


class Uart0 {
protected:
    static SerialPort port_;

public:
    static inline auto get_impl() {
        return &port_;
    }
};

} /* namespace avr */

#endif /* SRC_SERIAL_PORT_H_ */
