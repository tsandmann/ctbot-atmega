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
 * @file    motor.h
 * @brief   c't-Bot motor driver
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#ifndef SRC_MOTOR_H_
#define SRC_MOTOR_H_

#include "encoder.h"

#include <cstdint>


namespace ctbot {

/**
 * @brief Motor driver
 */
class Motor {
protected:
    static constexpr uint16_t PWM_PRESCALER { 1U }; /**< Clock prescaler for pwm timer */
    static constexpr uint16_t PWM_FREQUENCY { 16129U }; /**< Pwm frequency in Hz */
    static constexpr uint16_t PWM_TOP { static_cast<uint16_t>(static_cast<float>(F_CPU) / (1.f * PWM_PRESCALER * PWM_FREQUENCY)) }; /**< Timer top value to achieve requested PWM frequency */

    int16_t pwm_;
    volatile uint16_t* const p_ocr_;
    volatile uint8_t* const p_dir_port_;
    const uint8_t dir_pin_;
    const bool invert_dir_;
    Encoder& enc_;

public:
    /**
     * @brief Construct a new Motor object
     * @param[in] enc: Reference to encoder sensor of this motor (used to set direction)
     * @param[in] ddr_reg_pwm: Pointer to pin direction register for pwm pin, only used for initialization (set to output)
     * @param[in] pin_pwm: Pin number of the pwm signal, only used for initialization
     * @param[in] port_reg_dir: Pointer to port register for direction pin
     * @param[in] ddr_reg_dir: Pointer to pin direction register for direction pin, only used for initialization (set to output)
     * @param[in] pin_dir: Pin number of the direction signal
     * @param[in] invert: Invert motor direction setting; set to true, if wheel turning direction should be inverted
     * @param[in] ocr_reg: Pointer to output compare register of timer for pwm
     */
    Motor(Encoder& enc, volatile uint8_t* ddr_reg_pwm, uint8_t pin_pwm, volatile uint8_t* port_reg_dir, volatile uint8_t* ddr_reg_dir,
        uint8_t pin_dir, const bool invert, volatile uint16_t* ocr_reg);

    /**
     * @brief Set a new pwm duty cycle
     * @param[in] pwm: New pwm duty cycle to set; [- CtBotConfig::MOT_PWM_MAX; CtBotConfig::MOT_PWM_MAX]
     */
    void set(int16_t pwm);

    /**
     * @brief Set a new pwm duty cycle relative to max speed
     * @param[in] pwm_rel: New pwm duty cycle as ratio of max speed; [-1; +1]
     */
    void set(float pwm_rel) {
        set(static_cast<int16_t>(pwm_rel * CtBotConfig::MOT_PWM_MAX));
    }

    /**
     * @return Current pwm duty cycle set; [- CtBotConfig::MOT_PWM_MAX; CtBotConfig::MOT_PWM_MAX]
     */
    auto get() const {
        return pwm_;
    }

    /**
     * @brief Print current status as debug output
     */
    void print_status() const;
};

} /* namespace ctbot */

#endif /* SRC_MOTOR_H_ */
