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

static constexpr uint16_t SERIAL_RX_BUFFER_SIZE { 128 }; /**< Size of uart receive buffer in byte */

/**
 * @brief Base class for AVR serial port to allow parameterized receive buffer size
 * @tparam RX_BUF_SIZE: Size of receive buffer in byte
 */
template <size_t RX_BUF_SIZE>
class SerialPortBase {
protected:
    using rx_buf_idx_t = typename std::conditional<RX_BUF_SIZE <= 256, uint8_t, uint16_t>::type;
    rx_buf_idx_t rx_buf_head_;
    rx_buf_idx_t rx_buf_tail_;
    uint8_t rx_buffer_[RX_BUF_SIZE];

public:
    /**
     * @brief Construct a new SerialPortBase object
     */
    SerialPortBase() : rx_buf_head_ { 0 }, rx_buf_tail_ { 0 } {}
};


/**
 * @brief AVR serial port implementation
 */
class SerialPort : public SerialPortBase<SERIAL_RX_BUFFER_SIZE> {
public:
    /**
     * @brief Construct a new Serial Port object
     */
    SerialPort() = default;

    /**
     * @brief Destroy the Serial Port object
     */
    ~SerialPort() = default;

    /**
     * @brief Initialize the uart controller
     * @param baud: Baud rate to use
     */
    void begin(const uint32_t baud) const;

    /**
     * @brief Disable the uart
     */
    void end();

    /**
     * @brief Wait for any outstanding transmissions to complete
     */
    void flush() const;

    /**
     * @brief Get the number of bytes available in receive buffer
     * @return Number of bytes available
     */
    uint16_t available() const {
        return (static_cast<uint16_t>(sizeof(rx_buffer_) + rx_buf_head_ - rx_buf_tail_)) % sizeof(rx_buffer_);
    }

    /**
     * @brief Get the next byte received without removing it from the receive buffer
     * @return The next byte or -1, if receive buffer is empty
     */
    int16_t peek() const {
        return rx_buf_head_ == rx_buf_tail_ ? -1 : static_cast<int16_t>(rx_buffer_[rx_buf_tail_]);
    }

    /**
     * @brief Get the next byte received
     * @return The next byte or -1, if receive buffer is empty
     */
    int16_t read();

    /**
     * @brief Get at most length bytes received
     * @param buffer: Pointer to buffer where the received bytes should be copied into
     * @param length: Number of bytes to read at most
     * @return Actual number of bytes read into buffer
     */
    int16_t read(void* buffer, const size_t length);

    /**
     * @brief Send a byte out to serial port
     * @param data: The byte to send
     * @return Number of bytes sent out (1 in this case)
     */
    int16_t write(const uint8_t data) const;

    /**
     * @brief Send length bytes out to serial port
     * @param buffer: Pointer to buffer for data to send
     * @param length: Number of bytes to send
     * @return Actual number of bytes sent out
     */
    int16_t write(const void* buffer, const size_t length) const;

    /**
     * @brief ISR for uart receive complete interrupt
     */
    inline void isr();
};


/**
 * @brief Wrapper for SerialPort instance of ATmega's UART0
 */
class Uart0 {
protected:
    static SerialPort port_;

public:
    /**
     * @brief Get the wrapped SerialPort instance
     * @return Pointer to SerialPort intance
     */
    static inline auto get_impl() {
        return &port_;
    }
};

} /* namespace avr */

#endif /* SRC_SERIAL_PORT_H_ */
