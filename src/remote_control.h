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
 * @file    remote_control.h
 * @brief   c't-Bot remote control
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#ifndef SRC_REMOTE_CONTROL_H_
#define SRC_REMOTE_CONTROL_H_

#include "rc5_int.h"

#include <functor.h>
#include <map>


namespace ctbot {

class RemoteControl {
protected:
    using func_t = Functor<bool(uint8_t)>;

    Rc5& rc5_;
    const uint8_t addr_;
    bool last_toggle_;
    uint8_t last_cmd_;
    std::map<uint8_t /*cmd*/, func_t /*function*/> key_mappings_;

    void change_speed(bool right, float diff) const;

public:
    RemoteControl(Rc5& rc5, uint8_t rc5_address);

    void register_cmd(uint8_t cmd, func_t&& func);

    void update();
};

} /* namespace ctbot */

#endif /* SRC_REMOTE_CONTROL_H_ */
