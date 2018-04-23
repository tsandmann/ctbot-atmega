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
 * @file    servo.cpp
 * @brief   Servo driver
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#include "servo.h"

#include <avr/io.h>
#include <avr/interrupt.h>


namespace ctbot {
using namespace avr;

bool Servo::initialized { false };

Servo::Servo(ID num, volatile uint8_t* p_ddr, uint8_t pin, volatile uint16_t* p_ocr_reg) : id { num }, p_ocr { p_ocr_reg }, position { POS_OFF } {
    SBI(p_ddr, pin); // set servo pin as output

    if (! initialized) {
        initialized = true;
        GPIOR0 = 0; // both servos shut off
        TCNT3 = 0; // TIMER3 init
        ICR3 = 40000UL;
        TIMSK3 = 0; // disable timer interrupts
        TIFR3 = BV_8(TOV3) | BV_8(OCF3A) | BV_8(OCF3B) | BV_8(ICF3); // clear timer flags
        TCCR3A = 0; // normal port operation, OC3A/OC3B disconnected
        /* CTC mode, prescaler = 8 -> 20 MHz: 26.2144 ms period, 16 MHz: 32.768 ms period (without considering input compare register) */
        TCCR3B = BV_8(WGM32) | BV_8(WGM33) | BV_8(CS31);
    }
}

void Servo::set(uint8_t pos) {
    if (pos == POS_OFF) {
        /* set pwm pin low on next overflow */
        if (id == ID::SERVO_1) {
            CBI(&GPIOR0, static_cast<uint8_t>(0U));
        } else {
            CBI(&GPIOR0, static_cast<uint8_t>(1U));
        }
    } else {
        *p_ocr = calc_ocr(pos);
        if (id == ID::SERVO_1) {
            SBI(&GPIOR0, static_cast<uint8_t>(0U));
        } else {
            SBI(&GPIOR0, static_cast<uint8_t>(1U));
        }
        TIMSK3 |= BV_8(ICF3) | (id == ID::SERVO_1 ? BV_8(OCIE3A) : BV_8(OCIE3B)); // Input Capture Interrupt Enable, Output Compare A Match Interrupt Enable
    }
    position = pos;
}


/**
 * @brief Timer3 Input Capture Interrupt
 */
ISR(TIMER3_CAPT_vect, ISR_NAKED) {
    /* check for servo 1 active */
    if (GPIOR0 & (1 << static_cast<uint8_t>(Servo::ID::SERVO_1))) {
        SBI<uint8_t>(CtBotConfig::SERVO_1_REG::PORT, CtBotConfig::SERVO_1_PIN); // PWM0 high
    } else if (! (GPIOR0 & (1 << static_cast<uint8_t>(Servo::ID::SERVO_2)))) {
        /* both servos are off -> disable timer interrupts */
        __asm__ __volatile__ (
            "push r24		\n\t"
            "ldi r24, 0		\n\t"
            "sts %0, r24	\n\t"
            "pop r24			"
            ::	"n" (&TIMSK3)
            :	"memory"
        );
    }

    /* check for servo 2 active */
    if (GPIOR0 & (1 << static_cast<uint8_t>(Servo::ID::SERVO_2))) {
        SBI<uint8_t>(CtBotConfig::SERVO_2_REG::PORT, CtBotConfig::SERVO_2_PIN); // PWM2 high
    }

    reti();
}

/**
 * @brief Timer3 Output Compare A Match Interrupt
 */
ISR(TIMER3_COMPA_vect, ISR_NAKED) {
    CBI<uint8_t>(CtBotConfig::SERVO_1_REG::PORT, CtBotConfig::SERVO_1_PIN); // PWM0 low
    reti();
}

/**
 * @brief Timer3 Output Compare B Match Interrupt
 */
ISR(TIMER3_COMPB_vect, ISR_NAKED) {
    CBI<uint8_t>(CtBotConfig::SERVO_2_REG::PORT, CtBotConfig::SERVO_2_PIN); // PWM2 low
    reti();
}

} /* namespace ctbot */
