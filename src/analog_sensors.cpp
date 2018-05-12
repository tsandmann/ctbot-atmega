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
 * @file    analog_sensors.cpp
 * @brief   Analog sensor processing
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#include "analog_sensors.h"

#include <utils_avr.h>
#include <avr/io.h>


namespace ctbot {
using namespace avr;

AnalogSensors::AnalogSensors() {
    DDRA &= ~PIN_MASK; // set input
    PORTA &= ~PIN_MASK; // disable pullups
    DIDR0 = PIN_MASK; // disable digital inputs

    ADCSRA = BV_8(ADEN) | BV_8(ADPS2) | BV_8(ADPS1) | BV_8(ADPS0); // set prescaler to 128, enable ADC
}

void AnalogSensors::update() {
// FIXME: distance sensors every 50 ms only
    for (uint8_t i(CtBotConfig::DISTANCE_L_PIN); i <= CtBotConfig::BORDER_R_PIN; ++i) {
        data_.raw[i] = analog_read(i);
    }

    // data_.border[0] = analog_read(CtBotConfig::BORDER_L_PIN);
    // data_.border[1] = analog_read(CtBotConfig::BORDER_R_PIN);
    // data_.distance[0] = analog_read(CtBotConfig::DISTANCE_L_PIN); // FIXME: every 50 ms
    // data_.distance[1] = analog_read(CtBotConfig::DISTANCE_R_PIN); // FIXME: every 50 ms
    // data_.ldr[0] = analog_read(CtBotConfig::LDR_L_PIN);
    // data_.ldr[1] = analog_read(CtBotConfig::LDR_R_PIN);
    // data_.line[0] = analog_read(CtBotConfig::LINE_L_PIN);
    // data_.line[1] = analog_read(CtBotConfig::LINE_R_PIN);
}

int16_t AnalogSensors::analog_read(const uint8_t pin) const {
    // set the analog reference (high two bits of ADMUX) and select the channel (low 4 bits).
    // this also sets ADLAR (left-adjust result) to 0 (the default).
    ADMUX = BV_8(REFS0) | (pin & 0x7);

    // start the conversion
    SBI(&ADCSRA, ADSC);

    // ADSC is cleared when the conversion finishes
    while (ADCSRA & BV_8(ADSC)) {}

    // read ADCL first; doing so locks both ADCL and ADCH until ADCH is read
    const uint8_t low { ADCL };
    const uint8_t high { ADCH };

    // combine the two bytes
    return (high << 8) | low;
}

} /* namespace ctbot */
