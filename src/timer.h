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
 * @file    timer.h
 * @brief   Timer helper functions
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#ifndef SRC_TIMER_H_
#define SRC_TIMER_H_

#include "ctbot_config.h"

#include <utils_avr.h>
#include <cstdint>
#include <avr/io.h>


namespace ctbot {
using namespace avr;

/**
 * @brief Helper class to wrapp tick counter
 */
class TimerIsrHelper {
protected:
    static uint32_t tickcount_; /**< Tick counter */

public:
    /**
     * @brief Get the tickcount object
     * @return Pointer to tick counter
     */
    static inline auto get_tickcount() {
        return &tickcount_;
    }
};

/**
 * @brief Class to group timer helper functions
 *
 * @startuml{Timer.png}
 *   class Timer {
 *     +{static} get_tickcounter() : T
 *     +{static} get_timer() : constexpr volatile uint8_t*
 *     +{static} ticks_to_us(const uint32_t ticks, const uint8_t timer) : constexpr uint32_t
 *     +{static} us_to_ticks(const uint32_t us) : constexpr uint32_t
 *     +{static} get_us(T& ticks) : uint32_t
 *     +{static} get_us() : uint32_t
 *     +{static} get_ms() : uint32_t
 *     #{static} TIMER_REG : static constexpr intptr_t
 *   }
 * @enduml
 */
class Timer {
protected:
    static constexpr uint32_t TICK_RATE_HZ { 1250UL }; /**< Timer0 frequency in Hz */
    static constexpr uint32_t CLOCK_PRESCALER { 64UL }; /**< Prescaler for Timer0 (divider for CPU clock) */
    static constexpr intptr_t TIMER_REG { CtBotConfig::TCN_T0 }; /**< Address of counter register for Timer0 */
    static constexpr uint8_t CLEAR_COUNTER_ON_MATCH { BV_8(CtBotConfig::WGM_01) }; /**< Bit mask to set clear on compare match config of Timer0 */
    static constexpr uint8_t PRESCALE_64 { BV_8(CtBotConfig::CS_00) | BV_8(CtBotConfig::CS_01) }; /**< Bit mask to set prescaler of Timer0 to 64 */
    static constexpr uint8_t COMPARE_MATCH_A_INTERRUPT_ENABLE { BV_8(CtBotConfig::OCIE_0A) }; /**< Bit mask to enable compare match interrupt of Timer0 */

public:
    /**
     * @brief Initialize Timer
     * @note Timer0 is used as timer
     */
    static void init();

    /**
     * @brief Convert timer ticks to microsecods
     * @param[in] ticks: Timer ticks to convert
     * @param[in] timer: Optional timer counter register value to improve accurancy
     * @return Time in microseconds
     */
    static constexpr uint32_t ticks_to_us(const uint32_t ticks, const uint8_t timer = 0) {
        return ticks * static_cast<uint32_t>(1000000UL / TICK_RATE_HZ) + static_cast<uint32_t>(timer) * (CLOCK_PRESCALER * 1000000UL / F_CPU);
    }

    /**
     * @brief Convert timer ticks to millisecods
     * @param[in] ticks: Timer ticks to convert
     * @return Time in milliseconds
     */
    static constexpr uint32_t ticks_to_ms(const uint32_t ticks) {
        return ticks * 1000UL / TICK_RATE_HZ;
    }

    /**
     * @brief Convert microsenconds to timer ticks
     * @param[in] us: Microseconds to convert
     * @return Timer ticks
     */
    static constexpr uint32_t us_to_ticks(const uint32_t us) {
        return us / static_cast<uint32_t>(1000000UL / TICK_RATE_HZ);
    }

    /**
     * @brief Convert millisenconds to timer ticks
     * @param[in] ms: Milliseconds to convert
     * @return Timer ticks
     */
    static constexpr uint32_t ms_to_ticks(const uint32_t ms) {
        return ms * TICK_RATE_HZ / 1000UL;
    }

    /**
     * @brief Get the timer tick counter value
     * @tparam T: Type of pointer to tick counter
     * @return Current tick counter value
     */
    template <typename T = uint32_t, bool FROM_ISR = false>
    static inline T get_tickcount() {
        union {
            uint32_t* p_u32;
            volatile T* buf;
        } tmp;
        tmp.p_u32 = TimerIsrHelper::get_tickcount();

        ExecuteAtomic<FROM_ISR> x;
        return x([&tmp] () {
            const T ticks { *tmp.buf };
            return ticks;
        });
    }

    /**
     * @brief Get the counter reg of the timer
     * @return Pointer to timer counter register
     */
    static constexpr auto get_timer() {
        return PTR_8(TIMER_REG);
    }

    /**
     * @brief Get the current time in microseconds and timer ticks
     * @param[out] ticks: Reference to store current ticks value
     * @return Current time in us
     */
    template <typename T = uint32_t>
    static uint32_t get_us(T& ticks) {
        ticks = get_tickcount<T>();
        return ticks_to_us(ticks, *get_timer());
    }

    /**
     * @brief Get the current time in microseconds
     * @return Current time in us
     */
    template <typename T = uint32_t>
    static uint32_t get_us() {
        return ticks_to_us(get_tickcount<T>(), *get_timer());
    }

    /**
     * @brief Get the current time in milliseconds
     * @return Current time in ms
     */
    static uint32_t get_ms();
};

} /* namespace ctbot */

#endif /* SRC_TIMER_H_ */
