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
class SerialProtocol;

enum class PrintBase : uint8_t {
    NONE 	= 0,
    BIN		= 2,
    OCT		= 8,
    DEC		= 10,
    HEX		= 16,
};

class CommInterface {
protected:
    friend class CtBot;

    static constexpr size_t INPUT_BUFFER_SIZE { 128U };
    static constexpr uint16_t TASK_PERIOD_MS { 50U }; /**< Scheduling period of task in ms */

    SerialConnectionAVR& io_;
    bool echo_;
    int error_;
    char* p_input_;
    char input_buffer_[INPUT_BUFFER_SIZE];

    virtual void run() = 0;

    int16_t print_uint(const uint32_t v, const PrintBase base = PrintBase::DEC) const;

    int16_t print_int(const int32_t v, const PrintBase base = PrintBase::DEC) const;

public:
    CommInterface(SerialConnectionAVR& io_connection, bool enable_echo = false);

    auto get_echo() const {
        return echo_;
    }

    auto get_error() const {
        return error_;
    }

    void reset_error() {
        error_ = 0;
    }

    virtual void set_echo(bool value) = 0;

    int16_t debug_print(const avr::FlashStringHelper* str) const;

    int16_t debug_print(const char c) const;

    int16_t debug_print(const char* str) const;

    int16_t debug_print(const std::string& str) const;

    int16_t debug_print(const float v, const uint8_t digits) const;

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


class CommInterfaceCmdParser : public CommInterface {
protected:
    CmdParser& cmd_parser_;

    virtual void run() override;

public:
    CommInterfaceCmdParser(SerialConnectionAVR& io_connection, CmdParser& parser, bool enable_echo = false);

    virtual void set_echo(bool value) override;
};

} /* namespace ctbot */

#endif /* SRC_COMM_INTERFACE_H_ */
