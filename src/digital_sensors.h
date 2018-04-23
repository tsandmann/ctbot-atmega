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
 * @file    digital_sensors.h
 * @brief   Digital sensor processing
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#ifndef SRC_DIGITAL_SENSORS_H_
#define SRC_DIGITAL_SENSORS_H_

#include "ena.h"
#include "encoder.h"
#include "rc5_int.h"
#include "remote_control.h"
#include "ctbot_config.h"

#include <cstdint>


namespace ctbot {

class DigitalSensors {
protected:
    static constexpr auto ENA_MASK = EnaTypes::SHUTTER | EnaTypes::TRANSPORT;

    bool shutter_;
    bool transport_;
    bool error_;

    Encoder enc_l_;
    Encoder enc_r_;
    Rc5 rc5_;
    RemoteControl remote_control_;

    void update();

public:
    static Encoder::enc_t enc_data_l_[], enc_data_r_[];
    static volatile uint8_t enc_l_idx_, enc_r_idx_;

    DigitalSensors();

    auto get_shutter() const { return shutter_; }
    auto get_transport() const { return transport_; }
    auto get_error() const { return error_; }
    auto& get_enc_l() { return enc_l_; }
    auto& get_enc_r() { return enc_r_; }
    auto& get_rc5() { return rc5_; }
    auto& get_rc() { return remote_control_; }
};

} /* namespace ctbot */

#endif /* SRC_DIGITAL_SENSORS_H_ */
