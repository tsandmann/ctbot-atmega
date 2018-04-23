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

#include "cmd_parser.h"
#include "comm_interface.h"

#include <cstdlib>
#include <avr/pgmspace.h>


namespace ctbot {

CmdParser::CmdParser() : echo_(false) {}

void CmdParser::register_cmd(const avr::FlashStringHelper* cmd, const func_t& func) {
    auto const p_cmd(reinterpret_cast<const char*>(cmd));
    const auto len(strlen_P(p_cmd));
    if (len > MAX_CMD_LENGTH) {
        return;
    }

    auto buffer(new char[len + 1]);
    strcpy_P(buffer, p_cmd);
    commands_[std::string(buffer, len)] = func;

    // std::cout << "CmdParser::register_cmd(): \"" << buffer << "\" registered\n";

    delete buffer;
}


void CmdParser::register_cmd(const avr::FlashStringHelper* cmd, const char cmd_short, const func_t& func) {
    register_cmd(cmd, func);

    commands_[std::string(&cmd_short, 1)] = func;
}

bool CmdParser::execute_cmd(const std::string& cmd) {
    if (! cmd.length()) {
        return false;
    }

    // std::cout << "CmdParser::execute_cmd(): searching for cmd \"" << cmd << "\" ...\n";

    const auto arg_pos(cmd.find(" "));
    const auto cmd_str(cmd.substr(0, arg_pos));

    const auto it(commands_.find(cmd_str));
    if (it == commands_.end()) {
        return false;
    }

    std::string args;
    if (arg_pos != std::string::npos && cmd.length() > arg_pos + 1) {
        args = cmd.substr(arg_pos + 1);
    }

    // std::cout << "CmdParser::execute_cmd(): found, args=\"" << args << "\"\n";
    // std::cout << "CmdParser::execute_cmd(): cmd=\"" << it->first << "\" -> 0x" << std::hex
    //     << reinterpret_cast<const intptr_t>(&(it->second)) << std::dec << "\n";

    bool res { it->second(args) };

    // std::cout << "CmdParser::execute_cmd(): executed, res=" << res << "\n";

    return res;
}

bool CmdParser::parse(const char* in, CommInterface& comm) {
    static std::string last_line;

    if (*in) {
        last_line = in;
    }

    if (execute_cmd(last_line)) {
        if (echo_) {
            comm.debug_print(F("OK\n"));
        }
    } else {
        if (echo_) {
            comm.debug_print(F("ERROR\n"));
        }
    }

    return true;
}

} /* namespace ctbot */
