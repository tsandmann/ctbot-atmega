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
 * @file    encoder.h
 * @brief   c't-Bot wheel encoder driver
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#ifndef SRC_ENCODER_H_
#define SRC_ENCODER_H_

#include "timer.h"

#include <cstdint>


namespace ctbot {

/**
 * @brief Wheel encoder sensor processing
 */
class Encoder {
    friend class DigitalSensors; // for call of Encoder::init_interrupts() from DigitalSensors::DigitalSensors()

protected:
    struct enc_t {
        uint8_t ticks; /**< timer ticks (low 8 bits) */
        uint8_t timer; /**< value of timer register */
    } __attribute__((packed)); /**< Internal datatype for raw input data */

    static constexpr float AVG_FILTER_PARAM { 0.25f }; /**< Filter coefficient for speed averaging filter */
    static constexpr float WHEEL_PERIMETER { 178.1283 }; /**< Perimeter of the wheel in mm */
    static constexpr uint8_t ENCODER_MARKS { 60 }; /**< Number of encoder marks on the wheel */

    int16_t edges_; /**< Current number of edges counted; increasing for forward wheel turning, decreasing otherwise */
    uint8_t last_idx_; /**< Index in input data array of last processed entry */
    float speed_; /**< Current speed of wheel in mm/s */
    float speed_avg_; /**< Current speed of wheel as average in mm/s, @see AVG_FILTER_PARAM */
    bool direction_; /**< Current direction set for wheel turning; true: forward, false: backwards */

    /**
     * @brief Setup interrupt settings for pin change interrupts of (both) wheel encoders
     */
    static void init_interrupts();

    const enc_t* const p_enc_data_; /**< Pointer to data array to use where the raw input data is stored */
    const volatile uint8_t* const p_enc_idx_; /**< Pointer to current index in data array */
    uint32_t last_update_; /**< Timestamp of last edge processing */
    int8_t count_; /**< Internal counter for number of edges since last update */

public:
    static constexpr uint8_t DATA_ARRAY_SIZE { 8 }; /**< Size of buffer array in byte for raw encoder data */

    /**
     * @brief Construct a new Encoder object
     * @param[in] p_data: Pointer to data array to use where the raw input data is stored
     * @param[in] p_idx: Pointer to current index in data array
     * @param[in] p_ddr: Pointer to pin direction register, only used for initialization (set to input)
     * @param[in] p_port: Pointer to port register, only used for initialization (disable pullup)
     * @param[in] pin: Pin number of the input data signal, only used for initialization
     */
    Encoder(const enc_t* p_data, const volatile uint8_t* p_idx, volatile uint8_t* p_ddr, volatile uint8_t* p_port, const uint8_t pin);

    /**
     * @brief Check for new input data and calculate current speed
     */
    void update();

    /**
     * @return Number of measured edges
     */
    auto get() const {
        return edges_;
    }

    /**
     * @return Current speed average in mm/s
     */
    auto get_speed() const {
        return speed_avg_;
    }

    /**
     * @brief Set direction of wheel turning
     * @param[in] dir: true, if wheel is turning forwards, false otherwise
     */
    void set_direction(const bool dir) {
        direction_ = dir;
    }

    /**
     * @brief ISR for wheel encoder pin change interrupt
     * @tparam VECT_NUM: Number of interrupt vector (to distinguish ISRs)
     * @tparam ARRAY_SIZE: Size of raw input data array in byte
     */
    template <uint8_t VECT_NUM, uint8_t ARRAY_SIZE>
    static inline __attribute__((always_inline)) void isr(enc_t* p_data, volatile uint8_t* p_idx, const uint8_t timer) {
        const auto now(Timer::get_tickcount<uint8_t, true>());

        uint8_t idx(*p_idx);
        ++idx;
        idx %= ARRAY_SIZE;
        *p_idx = idx;
        p_data[idx].timer = timer;
        p_data[idx].ticks = now;
    }
};

} /* namespace ctbot */

#endif /* SRC_ENCODER_H_ */
