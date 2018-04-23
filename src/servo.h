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
 * @file    servo.h
 * @brief   Servo driver
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#ifndef SRC_SERVO_H_
#define SRC_SERVO_H_

#include "ctbot_config.h"

#include <utils_avr.h>
#include <cstdint>


namespace ctbot {
using namespace avr;

/**
 * @brief Servo management
 */
class Servo {
public:
    /**
     * @brief Enum of possible servo IDs
     */
    enum class ID : uint8_t {
        SERVO_1 = 0,
        SERVO_2 = 1,
    };

    static constexpr uint8_t POS_OFF { 0 };
    static constexpr uint8_t ACTIVE_BIT { 2 };

    /**
     * @brief Creates a servo instance for given servo number
     * @param[in] num id number of servo (ID::SERVO_1 or ID::SERVO_2)
     */
    Servo(ID num) : Servo(num, num == ID::SERVO_1 ? PTR_8(CtBotConfig::SERVO_1_REG::DDR) : PTR_8(CtBotConfig::SERVO_2_REG::DDR),
        num == ID::SERVO_1 ? CtBotConfig::SERVO_1_PIN : CtBotConfig::SERVO_2_PIN, num == ID::SERVO_1 ? PTR_16(CtBotConfig::SERVO_1_OCR) : PTR_16(CtBotConfig::SERVO_2_OCR)) {}

    /**
     * @brief Sets the servo to a position
     * @param[in] pos target position [1; 255] or POS_OFF to turn servo off
     */
    void set(uint8_t pos);

    /**
     * @brief Gets the last set servo position
     * @return position of servo or POS_OFF, if servo turned off
     */
    auto get() const { return position; }

protected:
    const ID id; /**< servo id, ID::SERVO_1 or ID::SERVO_2 */
    volatile uint16_t* const p_ocr;
    uint8_t position; /**< target position for servo */
    static bool initialized;

    /**
     * @brief Creates a servo instance for given servo number
     * @param[in] num id number of servo (ID::SERVO_1 or ID::SERVO_2)
     * @param[in] p_ddr pointer to pin direction register for this servo
     * @param[in] pin pin used for the servo
     * @param[in] p_ocr_reg pointer to OCR timer register for this servo
     */
    Servo(ID num, volatile uint8_t* p_ddr, uint8_t pin, volatile uint16_t* p_ocr_reg);

    /**
     * @brief Calculates the pwm pulse width for a given servo position
     * @param[in] pos servo position to use [1; 255]
     * @return pwm pulse width in us
     */
    static constexpr uint16_t calc_pulse(uint8_t pos) {
        return static_cast<uint16_t>(pos) * 7 + 600;
    }

    /**
     * @brief Calculates the value for the Timer Output Compare Match register
     * @param[in] pos servo position to use [1; 255]
     * @return Timer OC value for OCR register
     */
    static constexpr uint16_t calc_ocr(uint8_t pos) {
        return static_cast<uint16_t>(8.f * TIMER_MAX / (1000000.f / (F_CPU / TIMER_PRESCALER / TIMER_FACTOR / TIMER_MAX))) * calc_pulse(pos) / 8U;
    }

private:
    static constexpr float TIMER_PRESCALER { 8.f };
    static constexpr float TIMER_MAX { 65535.f };
    static constexpr float TIMER_FACTOR { 1.f };
};

} /* namespace ctbot */

#endif /* SRC_SERVO_H_ */
