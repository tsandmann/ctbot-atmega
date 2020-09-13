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

#ifndef SRC_SERIAL_CONNECTION_AVR_H_
#define SRC_SERIAL_CONNECTION_AVR_H_

#include <serial_port.h>
#include <streambuf>


namespace ctbot {

/**
 * @brief Abstraction layer for serial communication on AVR devices
 */
class SerialConnectionAVR {
    // FIXME: maybe this class can be simplified a lot...
protected:
    avr::SerialPort& io_stream_;

    static void (*wait_callback_)(const void*);

    uint16_t wait_for_data(const uint16_t size, const uint16_t timeout_ms);

public:
    /**
     * @brief Construct a new SerialConnectionAVR object
     * @param[in] io: Reference to underlying SerialPort instance
     */
    SerialConnectionAVR(avr::SerialPort& io) : io_stream_(io) {}

    /**
     * @brief Destroy the SerialConnectionAVR object
     */
    ~SerialConnectionAVR() = default;

    /**
     * @brief Copy constructor
     * @note Deleted to prohibit copies of this instance
     */
    SerialConnectionAVR(const SerialConnectionAVR&) = delete;

    /**
     * @brief Copy assignment operator
     * @return Reference to this instance
     * @note Deleted to prohibit copies of this instance
     */
    SerialConnectionAVR& operator=(const SerialConnectionAVR&) = delete;

    /**
     * @brief Set a callback function which is called in case of waiting for incoming data
     * @param[in] callback: Function pointer to callback function
     */
    static void set_wait_callback(decltype(wait_callback_) callback) {
        wait_callback_ = callback;
    }

    /**
     * @return Number of bytes available in receive buffer
     */
    std::size_t available() const;

    /**
     * @brief Read size bytes from connection into a plain buffer
     * @param[out] data: Pointer to buffer to read the data in
     * @param[in] size: Number of bytes to read
     * @return Number of bytes actually read
     */
    std::size_t receive(void* data, const std::size_t size);

    /**
     * @brief Read size bytes from connection into a streambuffer
     * @param[out] buf: Reference to streambuffer to read into
     * @param[in] size: Number of bytes to read
     * @return Number of bytes actually read
     */
    std::size_t receive(std::streambuf& buf, const std::size_t size);

    /**
     * @brief Read bytes from connection until a delimiter char into a plain buffer
     * @param[out] data: Pointer to buffer to read the data in
     * @param[in] delim: Delimiter which is used as a stop character for reading
     * @param[in] maxsize: Number of bytes to read at most
     * @return Number of bytes actually read
     */
    std::size_t receive_until(void* data, const char delim, const std::size_t maxsize);

    /**
     * @brief Read bytes from connection until a delimiter string into a plain buffer
     * @param[out] data: Pointer to buffer to read the data in
     * @param[in] delim: Reference to a delimiter string which is used as a stop sequence for reading
     * @param[in] maxsize: Number of bytes to read at most
     * @return Number of bytes actually read
     */
    std::size_t receive_until(void* data, const std::string& delim, const std::size_t maxsize);

    /**
     * @brief Read bytes from connection until a delimiter char into a streambuffer
     * @param[out] buf: Reference to streambuffer to read into
     * @param[in] delim: Delimiter which is used as a stop character for reading
     * @param[in] maxsize: Number of bytes to read at most
     * @return Number of bytes actually read
     */
    std::size_t receive_until(std::streambuf& buf, const char delim, const std::size_t maxsize);

    /**
     * @brief Read bytes from connection until a delimiter string into a streambuffer
     * @param[out] buf: Reference to streambuffer to read into
     * @param[in] delim: Reference to a delimiter string which is used as a stop sequence for reading
     * @param[in] maxsize: Number of bytes to read at most
     * @return Number of bytes actually read
     */
    std::size_t receive_until(std::streambuf& buf, const std::string& delim, const std::size_t maxsize);

    /**
     * @brief Read size bytes from connection into a plain buffer with a timeout
     * @param[out] data: Pointer to buffer to read the data in
     * @param[in] size: Number of bytes to read
     * @param[in] timeout_ms: Timeout while waiting for data
     * @return Number of bytes actually read
     */
    std::size_t receive_async(void* data, const std::size_t size, const uint32_t timeout_ms);

    /**
     * @brief Read size bytes from connection into a streambuffer with a timeout
     * @param[out] buf: Reference to streambuffer to read into
     * @param[in] size: Number of bytes to read at most
     * @param[in] timeout_ms: Timeout while waiting for data
     * @return Number of bytes actually read
     */
    std::size_t receive_async(std::streambuf& buf, const std::size_t size, const uint32_t timeout_ms);

    /**
     * @brief Write size bytes to connection from a plain buffer
     * @param[in] data: Pointer to buffer with data to write out
     * @param[in] size: Number of bytes to write
     * @return Number of bytes actually written
     */
    std::size_t send(const void* data, const std::size_t size);

    /**
     * @brief Write size bytes to connection from a streambuffer
     * @param[in] buf: Reference to streambuffer to read from
     * @param[in] size: Number of bytes to write
     * @return Number of bytes actually written
     */
    std::size_t send(std::streambuf& buf, const std::size_t size);

    /**
     * @brief Get the next byte received without removing it from the receive buffer
     * @return The next byte or -1, if receive buffer is empty
     * @note Just calls peek() of the underlying connection stream
     */
    int peek() const;
};

} /* namespace ctbot */

#endif /* SRC_SERIAL_CONNECTION_AVR_H_ */
