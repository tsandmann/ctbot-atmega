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
 * @file    sensors.h
 * @brief   c't-Bot sensor abstraction layer
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#ifndef SRC_SENSORS_H_
#define SRC_SENSORS_H_

#include "ena.h"
#include "digital_sensors.h"
#include "analog_sensors.h"
#include "timer.h"


namespace ctbot {

/**
 * @brief Collection of all c't-Bot sensors
 */
class Sensors : public DigitalSensors, public AnalogSensors {
protected:
    Ena ena_;

public:
    /**
     * @brief Construct a new Sensors object
     */
    Sensors();

    /**
     * @brief Update all sensors by calling the underlying update()-methods
     */
    void update();

    /**
     * @brief Disable all sensors by disabling their enable transistor
     */
    void disable_all() {
        ena_.set(EnaTypes::NONE);
    }

    /**
     * @return The current time in us
     */
    uint32_t get_time() const {
        return Timer::get_us();
    }
};

} /* namespace ctbot */

#endif /* SRC_SENSORS_H_ */
