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
 * @file    timer.cpp
 * @brief   Timer helper functions
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#include "timer.h"

#include <avr/interrupt.h>


namespace ctbot {

uint32_t TimerIsrHelper::tickcount_ { 0UL };

void Timer::init() {
    ExecuteAtomic<> x;
    x([]() {
        uint32_t ulCompareMatch { F_CPU / TICK_RATE_HZ };
        ulCompareMatch /= CLOCK_PRESCALER;
        ulCompareMatch -= 1UL;
        const uint8_t ucLowByte { static_cast<uint8_t>(ulCompareMatch & 0xffUL) };

        *PTR_8(TIMER_REG) = 0;
        TCCR0A = CLEAR_COUNTER_ON_MATCH;
        TCCR0B = PRESCALE_64;
        TIMSK0 = COMPARE_MATCH_A_INTERRUPT_ENABLE;

        OCR0A = ucLowByte;
    });
}

uint32_t Timer::get_ms() {
    return ticks_to_us(get_tickcount<uint32_t>(), *get_timer()) / 1000UL;
}

ISR(TIMER0_COMPA_vect) __attribute__((hot, flatten));
ISR(TIMER0_COMPA_vect) {
    (*TimerIsrHelper::get_tickcount())++;
}

} /* namespace ctbot */
