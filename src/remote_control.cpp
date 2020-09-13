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
 * @file    remote_control.cpp
 * @brief   c't-Bot remote control
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#include "remote_control.h"
#include "ctbot.h"
#include "sensors.h"
#include "speed_control.h"


namespace ctbot {

RemoteControl::RemoteControl(Rc5& rc5, uint8_t rc5_address) : rc5_(rc5), addr_(rc5_address), last_toggle_(rc5.get_toggle()), last_cmd_(rc5.get_cmd()) {
    /* register actions for keys / RC5 codes */

    register_cmd(0x35 /* play */, [this](uint8_t) {
        /* Hello World */
        auto const p_ctbot(&CtBot::get_instance());
        p_ctbot->get_comm()->debug_print(F("Hello World!\n"));
        p_ctbot->get_comm()->debug_print(F("addr_=0x"));
        p_ctbot->get_comm()->debug_print(addr_, PrintBase::HEX);
        p_ctbot->get_comm()->debug_print(F("\n"));
        return true;
    });

    register_cmd(0xc /* power */, [](uint8_t) {
        /* stop motors */
        auto const p_ctbot(&CtBot::get_instance());
        p_ctbot->get_speedcontrols()[0]->set_speed(0.f);
        p_ctbot->get_speedcontrols()[1]->set_speed(0.f);
        return true;
    });

    register_cmd(0x29 /* pause */, [this](uint8_t) {
        /* increase speed left and right */
        this->change_speed(false, 10.f);
        this->change_speed(true, 10.f);
        return true;
    });

    register_cmd(0x36 /* stop */, [this](uint8_t) {
        /* decrease speed left and right */
        this->change_speed(false, -10.f);
        this->change_speed(true, -10.f);
        return true;
    });

    register_cmd(0x32 /* << */, [this](uint8_t) {
        /* increase speed right */
        this->change_speed(true, 10.f);
        return true;
    });

    register_cmd(0x34 /* >> */, [this](uint8_t) {
        /* increase speed left */
        this->change_speed(false, 10.f);
        return true;
    });

    register_cmd(0x2b /* I/II */, [](uint8_t) {
        /* shutdown */
        auto const p_ctbot(&CtBot::get_instance());
        p_ctbot->stop();
        return true;
    });
}

void RemoteControl::register_cmd(const uint8_t cmd, func_t&& func) {
    key_mappings_[cmd] = { std::move(func) };
}

void RemoteControl::update() {
    if (rc5_.get_addr() != addr_) {
        return;
    }

    const auto toggle(rc5_.get_toggle());
    const auto cmd(rc5_.get_cmd());

    if (last_toggle_ != toggle || last_cmd_ != cmd) {
        last_toggle_ = toggle;
        const auto it(key_mappings_.find(cmd));
        if (it == key_mappings_.end()) {
            return;
        }
        if (it->second(cmd)) {
            last_cmd_ = cmd;
        }
    }
}

void RemoteControl::change_speed(bool right, float diff) const {
    auto& ctbot(CtBot::get_instance());
    auto speed(ctbot.get_speedcontrols()[right ? 1 : 0]->get_speed());
    speed += diff;
    ctbot.get_speedcontrols()[right ? 1 : 0]->set_speed(speed);
}

} /* namespace ctbot */
