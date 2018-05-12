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
 * @file    cmd_parser.h
 * @brief   Command interface parser
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#ifndef SRC_CMD_PARSER_H_
#define SRC_CMD_PARSER_H_

#include <functor.h>
#include <utils_avr.h>
#include <cstdint>
#include <map>
#include <string>


namespace ctbot {

class CommInterface;

/**
 * @brief Simple parser for console commands
 */
class CmdParser {
protected:
    using func_t = Functor<bool(const std::string &)>;
    static constexpr size_t MAX_CMD_LENGTH { 16 };

    bool echo_;
    std::map<std::string /*cmd*/, func_t /*function*/> commands_;

    bool execute_cmd(const std::string& cmd);

public:
    /**
     * @brief Construct a new CmdParser object
     */
    CmdParser();

    /**
     * @brief Register a new command given as a C-string in program memory
     * @param[in] cmd: Pointer to C-string of command stored in program memory
     * @param[in] func: Functor representing the action to execute for the command (may be a lambda)
     */
    void register_cmd(const avr::FlashStringHelper* cmd, const func_t& func);

    /**
     * @brief Register a new command given as a C-string in program memory
     * @param[in] cmd: Pointer to C-string of command stored in program memory
     * @param[in] cmd_short: Shortcut for command as a single character
     * @param[in] func: Functor representing the action to execute for the command (may be a lambda)
     */
    void register_cmd(const avr::FlashStringHelper* cmd, const char cmd_short, const func_t& func);

    /**
     * @brief Parse the input data and execute the corresponding command, if registered
     * @param[in] in: Pointer to input data buffer as C-string
     * @param[in] comm: Reference to CommInterface (for debugging output only)
     * @return true on success
     */
    bool parse(const char* in, CommInterface& comm);

    /**
     * @brief Set the character echo mode for console
     * @param[in] value: true to activate character echo on console, false otherwise
     */
    void set_echo(bool value) {
        echo_ = value;
    }

    /**
     * @brief Split a string into space seperated tokens and return the first as integer argument
     * @tparam T: Type of argument to get out
     * @param[in] args: Reference to input string
     * @param[out] x1: Reference to first output argument
     * @return Pointer to the character past the last character interpreted
     */
    template <typename T>
    static char* split_args(const std::string& args, T& x1) {
        T x2;
        return split_args(args, x1, x2);
    }

    /**
     * @brief Split a string into space seperated tokens and return the first two as integer arguments
     * @tparam T: Type of argument to get out
     * @param[in] args: Reference to input string
     * @param[out] x1: Reference to first output argument
     * @param[out] x2: Reference to second output argument
     * @return Pointer to the character past the last character interpreted
     */
    template <typename T>
    static char* split_args(const std::string& args, T& x1, T& x2) {
// FIXME: implement as a variadic template?
        const auto l(args.find(" ") + 1);
        char* p_end;
        x1 = static_cast<T>(std::strtol(args.c_str() + l, &p_end, 10));
        x2 = static_cast<T>(std::strtol(p_end, &p_end, 10));
        return p_end;
    }
};

} /* namespace ctbot */

#endif /* SRC_CMD_PARSER_H_ */
