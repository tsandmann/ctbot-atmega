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

/**
 * @brief Abstraction layer for (simple) digital sensors
 */
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

    /**
     * @brief Read all the current pin values
     */
    void update();

public:
    static Encoder::enc_t enc_data_l_[], enc_data_r_[]; /**< Raw input data buffers for both wheel encoders */
    static volatile uint8_t enc_l_idx_, enc_r_idx_; /**< Current indices in input data buffers, pointing to each latest entry */

    /**
     * @brief Construct a new DigitalSensors object
     */
    DigitalSensors();

    /**
     * @return The last value of shutter sensor
     */
    auto get_shutter() const {
        return shutter_;
    }

    /**
     * @return The last value of transport pocket occupancy
     */
    auto get_transport() const {
        return transport_;
    }

    /**
     * @return The last value of error signal
     */
    auto get_error() const {
        return error_;
    }

    /**
     * @return Reference to the left wheel encoder driver
     */
    auto& get_enc_l() {
        return enc_l_;
    }

    /**
     * @return Reference to the right wheel encoder driver
     */
    auto& get_enc_r() {
        return enc_r_;
    }

    /**
     * @return Reference to the IR receiver driver
     */
    auto& get_rc5() {
        return rc5_;
    }

    /**
     * @return Reference to the RC5 remote control driver
     */
    auto& get_rc() {
        return remote_control_;
    }
};

} /* namespace ctbot */

#endif /* SRC_DIGITAL_SENSORS_H_ */
