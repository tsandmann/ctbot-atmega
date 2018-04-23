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
 * @file    encoder.cpp
 * @brief   c't-Bot wheel encoder driver
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#include "encoder.h"
#include "digital_sensors.h"

#include <utils_avr.h>
#include <iostream>
#include <avr/io.h>


namespace ctbot {
using namespace avr;

Encoder::Encoder(const enc_t* const p_data, const volatile uint8_t* const p_idx, volatile uint8_t* p_ddr, volatile uint8_t* p_port, uint8_t pin) :
        edges_(0), last_idx_(0), speed_(0.f), speed_avg_(0.f), direction_(true), p_enc_data_(p_data), p_enc_idx_(p_idx), last_update_(0), count_(0) {
    CBI(p_ddr, pin); // set input
    CBI(p_port, pin); // disable pullup
}

void Encoder::init_interrupts() {
// FIXME: avoid hard coded values where possible
    SBI(&EICRA, ISC10); // Any logical change on INT1 generates an interrupt request
    SBI(&EIMSK, INT1); // External Interrupt Request 1 Enable
    SBI(&EIFR, INTF1); // clear int flag

    if (CtBotConfig::ENC_L_REG::PORT == CtBotConfig::PORT_B::PORT && CtBotConfig::ENC_L_PIN == 4) {
        SBI(&PCMSK1, PCINT12); // Pin Change Enable Mask for PB4
        SBI(&PCICR, PCIE1); // Pin Change Interrupt Enable 1
        SBI(&PCIFR, PCIF1); // clear int flag
    }

    if (CtBotConfig::ENC_L_REG::PORT == CtBotConfig::PORT_C::PORT && CtBotConfig::ENC_L_PIN == 5) {
        SBI(&PCMSK2, PCINT21); // Pin Change Enable Mask for PC5
        SBI(&PCICR, PCIE2); // Pin Change Interrupt Enable 2
        SBI(&PCIFR, PCIF2); // clear int flag
    }
}

void Encoder::update() {
    const uint8_t idx(*p_enc_idx_);
    int8_t diff_enc(idx - last_idx_);
    if (diff_enc < 0) {
        diff_enc += DATA_ARRAY_SIZE;
    }
    // std::cout << static_cast<uint16_t>(idx) << "\t" << static_cast<uint16_t>(diff_enc) << "\n";

    uint16_t now;
    uint32_t now_us { Timer::get_us(now) };
    int32_t dt { static_cast<int32_t>(now_us - last_update_) };
    if (dt < 0) {
        dt += Timer::ticks_to_us(65536UL);
    }
    if (diff_enc) {
        if (! direction_) {
            diff_enc = -diff_enc;
        }
        edges_ += diff_enc;
        last_idx_ = idx;

        uint16_t ticks { p_enc_data_[idx].ticks };
        if (ticks <= (now & 0xff)) {
            ticks = (now & 0xff00) | ticks;
        } else {
            ticks = ((now - 256) & 0xff00) | ticks;
        }

        // std::cout << ticks << "\t" << static_cast<uint16_t>(diff_enc) << "\n";

        count_ += diff_enc;

        // std::cout << static_cast<int16_t>(count_) << "\n";

        const uint32_t current_time { Timer::ticks_to_us(ticks, p_enc_data_[idx].timer) };
        int32_t diff { static_cast<int32_t>(current_time - last_update_) };
        if (diff < 0) {
            diff += Timer::ticks_to_us(65536UL);
        }
        if (diff == 0) {
            return;
        }
        speed_ = (WHEEL_PERIMETER / ENCODER_MARKS * 1000000.f) * count_ / diff;
        speed_avg_ = speed_avg_ * (1.f - AVG_FILTER_PARAM) + speed_ * AVG_FILTER_PARAM;

        if (speed_ != 0.f) {
            // std::cout << speed_ << "\t" << speed_avg_ << "\t" << /* current_time << "\t" << last_update_ << "\t" << */ static_cast<uint16_t>(count_) << "\n";
            // for (auto i(0U); i < 8; ++i) {
            // 	std::cout << static_cast<uint16_t>(p_enc_data_[i].ticks) << "\t";
            // }
            // std::cout << static_cast<uint16_t>(idx) << "\n";
        }
        if (count_) {
            last_update_ = current_time;
        }
        count_ = 0;
    } else if (dt > 600000L) {
        // std::cout << now_us << "\t" << last_update_ << "\t" << dt << "\n";
        speed_ = speed_avg_ = 0.f;
        count_ = 0;
        last_update_ = now_us;
    }
}

} /* namespace ctbot */
