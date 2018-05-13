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
 * @file    comm_interface.h
 * @brief   Communication interface classes of c't-Bot atmega framework
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#ifndef SRC_COMM_INTERFACE_H_
#define SRC_COMM_INTERFACE_H_

#include <utils_avr.h>
#include <cstdint>
#include <string>
#include <type_traits>
#include <avr/pgmspace.h>


namespace ctbot {

class CtBot;
class CmdParser;
class SerialConnectionAVR;

/**
 * @brief Enum class for possible bases
 */
enum class PrintBase : uint8_t {
    NONE    = 0,
    BIN     = 2,
    OCT     = 8,
    DEC     = 10,
    HEX     = 16,
};

/**
 * @brief Abstraction layer for communication services
 */
class CommInterface {
protected:
    friend class CtBot;

    static constexpr size_t INPUT_BUFFER_SIZE { 128U }; /**< Size of input buffer in byte */
    static constexpr uint16_t TASK_PERIOD_MS { 50U }; /**< Scheduling period of task in ms */

    SerialConnectionAVR& io_;
    bool echo_;
    int error_;
    char* p_input_;
    char input_buffer_[INPUT_BUFFER_SIZE];

    /**
     * @brief Worker task implementation that processes incoming data
     * @note Pure virtual method, override for specialized implementations
     */
    virtual void run() = 0;

    int16_t print_uint(const uint32_t v, const PrintBase base = PrintBase::DEC) const;

    int16_t print_int(const int32_t v, const PrintBase base = PrintBase::DEC) const;

public:
    /**
     * @brief Construct a new CommInterface object
     * @param[in] io_connection: Reference to SerialConnection to use
     * @param[in] enable_echo: character echo mode for console, defaults to false
     */
    CommInterface(SerialConnectionAVR& io_connection, bool enable_echo = false);

    /**
     * @return Current character echo mode setting
     */
    auto get_echo() const {
        return echo_;
    }

    /**
     * @return Last error code
     */
    auto get_error() const {
        return error_;
    }

    /**
     * @brief Reset the stored error code
     */
    void reset_error() {
        error_ = 0;
    }

    /**
     * @brief Set the character echo mode for console
     * @param[in] value: true to activate character echo on console, false otherwise
     * @note Pure virtual method, override for specialized implementations
     */
    virtual void set_echo(bool value) = 0;

    /**
     * @brief Write a messages stored in program memory out to SerialConnection
     * @param[in] str: Pointer to message as C-string stored in program memory
     * @return Number of characters written
     */
    int16_t debug_print(const avr::FlashStringHelper* str) const;

    /**
     * @brief Write a character out to SerialConnection
     * @param[in] c: Character to write
     * @return Number of characters written
     */
    int16_t debug_print(const char c) const;

    /**
     * @brief Write a messages out to SerialConnection
     * @param[in] str: Pointer to message as C-string
     * @return Number of characters written
     */
    int16_t debug_print(const char* str) const;

    /**
     * @brief Write a messages out to SerialConnection
     * @param[in] str: Reference to message as string
     * @return Number of characters written
     */
    int16_t debug_print(const std::string& str) const;

    /**
     * @brief Write a floating point number out to SerialConnection
     * @param[in] v: Floating point value
     * @param[in] digits: Number of digits of floating point number to write
     * @return Number of characters written
     */
    int16_t debug_print(const float v, const uint8_t digits) const;

    /**
     * @brief Write an integer out to SerialConnection
     * @tparam T: Type of integer
     * @param[in] v: Integer value
     * @param[in] base: Base of positional numeral system to use
     * @return Number of characters written
     */
    template <typename T>
    int16_t debug_print(const T v, const PrintBase base) const {
        if (std::is_integral<T>::value && std::is_unsigned<T>::value) {
            return print_uint(v, base);
        }
        if (std::is_integral<T>::value && std::is_signed<T>::value) {
            return print_int(v, base);
        }
    }
};


/**
 * @brief Specialization of CommInterface that uses a CmdParser to build a simple command line interface
 */
class CommInterfaceCmdParser : public CommInterface {
protected:
    CmdParser& cmd_parser_;

    /**
     * @brief Worker task implementation that processes incoming data
     * @details The incoming data is parsed with the associated CmdParser and
     * registered commands are executed accordingly.
     */
    virtual void run() override;

public:
    /**
     * @brief Construct a new CommInterfaceCmdParser object
     * @param[in] io_connection: Reference to SerialConnection to use
     * @param[in] parser: Reference to CmdParser to use
     * @param[in] enable_echo: character echo mode for console, defaults to false
     */
    CommInterfaceCmdParser(SerialConnectionAVR& io_connection, CmdParser& parser, bool enable_echo = false);

    /**
     * @brief Set the character echo mode for console
     * @param[in] value: true to activate character echo on console, false otherwise
     */
    virtual void set_echo(bool value) override;
};

} /* namespace ctbot */

#endif /* SRC_COMM_INTERFACE_H_ */
