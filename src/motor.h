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

class Motor {
protected:
    static constexpr uint16_t PWM_PRESCALER { 1U };
    static constexpr uint16_t PWM_FREQUENCY { 16129U }; // PWM frequency / Hz
    static constexpr uint16_t PWM_TOP { static_cast<uint16_t>(static_cast<float>(F_CPU) / (1.f * PWM_PRESCALER * PWM_FREQUENCY)) };

    int16_t pwm_;
    volatile uint16_t* const p_ocr_;
    volatile uint8_t* const p_dir_port_;
    const uint8_t dir_pin_;
    const bool invert_dir_;
    Encoder& enc_;

public:
    Motor(Encoder& enc, volatile uint8_t* ddr_reg_pwm, uint8_t pin_pwm, volatile uint8_t* port_reg_dir, volatile uint8_t* ddr_reg_dir,
        uint8_t pin_dir, bool invert, volatile uint16_t* ocr_reg);

    void set(int16_t pwm);

    void set(float pwm_pc) {
        set(static_cast<int16_t>(pwm_pc * CtBotConfig::MOT_PWM_MAX));
    }

    auto get() const {
        return pwm_;
    }

    void print_status() const;
};

} /* namespace ctbot */

#endif /* SRC_MOTOR_H_ */
