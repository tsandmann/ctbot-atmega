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
 * @file    analog_sensors.h
 * @brief   Analog sensor processing
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#ifndef SRC_ANALOG_SENSORS_H_
#define SRC_ANALOG_SENSORS_H_

#include "ctbot_config.h"
#include "ena.h"

#include <utils_avr.h>
#include <cstdint>


namespace ctbot {
using namespace avr;

/**
 * @brief Abstraction layer for (simple) analog sensors
 * @note No further sensor data processing is done here, just the raw ADC values are collected.
 */
class AnalogSensors {
protected:
    static constexpr auto ENA_MASK = EnaTypes::BORDER | EnaTypes::LINE; // | EnaTypes::DISTANCE;
    static constexpr uint8_t PIN_MASK = BV_8(CtBotConfig::DISTANCE_L_PIN) | BV_8(CtBotConfig::DISTANCE_R_PIN) | BV_8(CtBotConfig::LINE_L_PIN)
        | BV_8(CtBotConfig::LINE_R_PIN) | BV_8(CtBotConfig::LDR_L_PIN) | BV_8(CtBotConfig::LDR_R_PIN) | BV_8(CtBotConfig::BORDER_L_PIN)
        | BV_8(CtBotConfig::BORDER_R_PIN);

    union {
        struct {
            uint16_t distance[2];
            uint16_t line[2];
            uint16_t ldr[2];
            uint16_t border[2];
        } __attribute__((packed));
        uint16_t raw[8];
    } data_;

    /**
     * @brief Read all the current ADC values
     */
    void update();

    /**
     * @brief Read the ADC value of a pin
     * @param[in] pin: The pin to read from
     * @return ADC value in 10 bit resolution
     */
    int16_t analog_read(const uint8_t pin) const;

public:
    /**
     * @brief Construct a new AnalogSensors object
     */
    AnalogSensors();

    /**
     * @return The last value of left border sensor
     */
    auto get_border_l() const {
        return data_.border[0];
    }

    /**
     * @return The last value of right border sensor
     */
    auto get_border_r() const {
        return data_.border[1];
    }

    /**
     * @return The last value of left distance sensor
     */
    auto get_distance_l() const {
        return data_.distance[0];
    }

    /**
     * @return The last value of right distance sensor
     */
    auto get_distance_r() const {
        return data_.distance[1];
    }

    /**
     * @return The last value of left LDR sensor
     */
    auto get_ldr_l() const {
        return data_.ldr[0];
    }

    /**
     * @return The last value of right LDR sensor
     */
    auto get_ldr_r() const {
        return data_.ldr[1];
    }

    /**
     * @return The last value of left line sensor
     */
    auto get_line_l() const {
        return data_.line[0];
    }

    /**
     * @return The last value of right line sensor
     */
    auto get_line_r() const {
        return data_.line[1];
    }
};

} /* namespace ctbot */

#endif /* SRC_ANALOG_SENSORS_H_ */
