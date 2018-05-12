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
 * @file    pinchange_isrs.cpp
 * @brief   Pin change ISR implementations
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#include "ctbot_config.h"
#include "digital_sensors.h"

#include <utils_avr.h>
#include <avr/io.h>
#include <avr/interrupt.h>


namespace ctbot {
using namespace avr;

/**
 * @brief External interrupt request 1 ISR
 */
ISR(INT1_vect) {
    const auto timer(*Timer::get_timer());

    /* right wheel encoder */
    Encoder::isr<INT1_vect_num, Encoder::DATA_ARRAY_SIZE>(DigitalSensors::enc_data_r_, &DigitalSensors::enc_r_idx_, timer);
}

/**
 * @brief Pin change interrupt request 1 ISR
 */
ISR(PCINT1_vect) {
    static bool rc5_last { true }, enc_l_last { false };
    const auto timer(*Timer::get_timer());

    /* RC5 decoder */
    if (CtBotConfig::RC5_REG::PORT == CtBotConfig::PORT_B::PORT && CtBotConfig::RC5_PIN == 1) {
        const bool rc5_value((*PTR_8(CtBotConfig::RC5_REG::PINR) >> CtBotConfig::RC5_PIN) & 1);
        if (rc5_value != rc5_last) {
            rc5_last = rc5_value;
            Rc5::isr<PCINT1_vect_num, Rc5::DATA_ARRAY_SIZE>(rc5_value, Rc5::input_data_, &Rc5::input_idx_, timer);
        }
    }

    /* left wheel encoder */
    if (CtBotConfig::ENC_L_REG::PORT == CtBotConfig::PORT_B::PORT && CtBotConfig::ENC_L_PIN != 2) {
        const bool enc_l_value((*PTR_8(CtBotConfig::ENC_L_REG::PINR) >> CtBotConfig::ENC_L_PIN) & 1);
        if (enc_l_value != enc_l_last) {
            enc_l_last = enc_l_value;
            Encoder::isr<PCINT1_vect_num, Encoder::DATA_ARRAY_SIZE>(DigitalSensors::enc_data_l_, &DigitalSensors::enc_l_idx_, timer);
        }
    }

    SBI(&PCIFR, PCIF1); // clear int flag, because we may have handled multiple pin changes
}

/**
 * @brief Pin change interrupt request 2 ISR
 */
ISR(PCINT2_vect) {
    static bool enc_l_last { false };

    const auto timer(*Timer::get_timer());

    /* left wheel encoder */
    if (CtBotConfig::ENC_L_REG::PORT == CtBotConfig::PORT_C::PORT) {
        const bool enc_l_value((*PTR_8(CtBotConfig::ENC_L_REG::PINR) >> CtBotConfig::ENC_L_PIN) & 1);
        if (enc_l_value != enc_l_last) {
            enc_l_last = enc_l_value;
            Encoder::isr<PCINT2_vect_num, Encoder::DATA_ARRAY_SIZE>(DigitalSensors::enc_data_l_, &DigitalSensors::enc_l_idx_, timer);
        }
    }
}

} /* namespace ctbot */
