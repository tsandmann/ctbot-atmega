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
 * @file    motor.cpp
 * @brief   c't-Bot motor driver
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#include "motor.h"
#include "ctbot.h"

#include <avr/io.h>


namespace ctbot {

Motor::Motor(Encoder& enc, volatile uint8_t* ddr_reg_pwm, uint8_t pin_pwm, volatile uint8_t* port_reg_dir,
        volatile uint8_t* ddr_reg_dir, uint8_t pin_dir, bool invert, volatile uint16_t* ocr_reg)
        : pwm_(0), p_ocr_(ocr_reg), p_dir_port_(port_reg_dir), dir_pin_(pin_dir), invert_dir_(invert), enc_(enc) {
    SBI(ddr_reg_pwm, pin_pwm); // pwm pin output
    SBI(ddr_reg_dir, pin_dir); // direction pin output

    /* clear on compare match, fast PWM, TOP at ICR1, prescaler 1 */
    TCCR1A = BV_8(COM1A1) | BV_8(COM1B1) | BV_8(WGM11);
    TCCR1B = BV_8(WGM13) | BV_8(WGM12) | BV_8(CS10);
    TCNT1 = 0;
    ICR1 = PWM_TOP;

    set(static_cast<int16_t>(0));
}

void Motor::set(int16_t new_pwm) {
    if (new_pwm > 0) {
        enc_.set_direction(true);
    } else if (new_pwm < 0) {
        enc_.set_direction(false);
    }

    pwm_ = abs(new_pwm) > CtBotConfig::MOT_PWM_MAX ? (new_pwm < 0 ? -CtBotConfig::MOT_PWM_MAX : CtBotConfig::MOT_PWM_MAX) : new_pwm;
    if (invert_dir_) {
        new_pwm = -new_pwm;
    }
    if (new_pwm >= 0) {
        SBI(p_dir_port_, dir_pin_);
    } else {
        CBI(p_dir_port_, dir_pin_);
    }
    *p_ocr_ = static_cast<uint16_t>(static_cast<float>(abs(pwm_)) * (static_cast<float>(PWM_TOP) / CtBotConfig::MOT_PWM_MAX));
}

void Motor::print_status() const {
    auto p_comm(CtBot::get_instance().get_comm());
    p_comm->debug_print(F("TCCR1A="));
    p_comm->debug_print(TCCR1A, PrintBase::HEX);
    p_comm->debug_print(F("\nTCCR1B="));
    p_comm->debug_print(TCCR1B, PrintBase::HEX);
    p_comm->debug_print(F("\nICR1="));
    p_comm->debug_print(ICR1, PrintBase::HEX);
    p_comm->debug_print(F("\nOCR1A="));
    p_comm->debug_print(OCR1A, PrintBase::HEX);
    p_comm->debug_print(F("\nOCR1B="));
    p_comm->debug_print(OCR1B, PrintBase::HEX);
    p_comm->debug_print('\n');
}

} /* namespace ctbot */
