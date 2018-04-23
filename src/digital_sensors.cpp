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
 * @file    digital_sensors.cpp
 * @brief   Digital sensor processing
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#include "digital_sensors.h"

#include <utils_avr.h>
#include <type_traits>
#include <avr/io.h>


namespace ctbot {
using namespace avr;

std::remove_all_extents<decltype(DigitalSensors::enc_data_l_)>::type DigitalSensors::enc_data_l_[Encoder::DATA_ARRAY_SIZE], DigitalSensors::enc_data_r_[Encoder::DATA_ARRAY_SIZE];
decltype(DigitalSensors::enc_l_idx_) DigitalSensors::enc_l_idx_, DigitalSensors::enc_r_idx_;

DigitalSensors::DigitalSensors() : shutter_(false), transport_(false), error_(false),
        enc_l_(enc_data_l_, &enc_l_idx_, PTR_8(CtBotConfig::ENC_L_REG::DDR), PTR_8(CtBotConfig::ENC_L_REG::PORT), CtBotConfig::ENC_L_PIN),
        enc_r_(enc_data_r_, &enc_r_idx_, PTR_8(CtBotConfig::ENC_R_REG::DDR), PTR_8(CtBotConfig::ENC_R_REG::PORT), CtBotConfig::ENC_R_PIN),
        rc5_(PTR_8(CtBotConfig::RC5_REG::DDR), PTR_8(CtBotConfig::RC5_REG::PORT), PTR_8(CtBotConfig::RC5_PCMSK), CtBotConfig::RC5_PIN, CtBotConfig::RC5_PCI),
        remote_control_(rc5_, CtBotConfig::RC5_ADDR) {
    CBI<uint8_t>(CtBotConfig::SHUTTER_REG::DDR, CtBotConfig::SHUTTER_PIN); // set input
    CBI<uint8_t>(CtBotConfig::SHUTTER_REG::PORT, CtBotConfig::SHUTTER_PIN); // disable pullup

    CBI<uint8_t>(CtBotConfig::TRANSPORT_REG::DDR, CtBotConfig::TRANSPORT_PIN); // set input
    CBI<uint8_t>(CtBotConfig::TRANSPORT_REG::PORT, CtBotConfig::TRANSPORT_PIN); // disable pullup

    CBI<uint8_t>(CtBotConfig::ERROR_REG::DDR, CtBotConfig::ERROR_PIN); // set input
    CBI<uint8_t>(CtBotConfig::ERROR_REG::PORT, CtBotConfig::ERROR_PIN); // disable pullup

    if (CtBotConfig::UART0_RX_PULLUP_ON) {
        SBI<uint8_t>(CtBotConfig::UART0_RX_REG::PORT, CtBotConfig::UART0_RX_PIN); // enable pullup on RX0
    } else {
        CBI<uint8_t>(CtBotConfig::UART0_RX_REG::PORT, CtBotConfig::UART0_RX_PIN); // disable pullup on RX0
    }

    Encoder::init_interrupts();
}

void DigitalSensors::update() {
    shutter_ = (*PTR_8(CtBotConfig::SHUTTER_REG::PINR) >> CtBotConfig::SHUTTER_PIN) & 1;
    transport_ = (*PTR_8(CtBotConfig::TRANSPORT_REG::PINR) >> CtBotConfig::TRANSPORT_PIN) & 1;
    error_ = (*PTR_8(CtBotConfig::ERROR_REG::PINR) >> CtBotConfig::ERROR_PIN) & 1;

    enc_l_.update();
    enc_r_.update();
    rc5_.update();
    remote_control_.update();
}

} /* namespace ctbot */


