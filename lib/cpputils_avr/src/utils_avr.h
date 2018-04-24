/*
 * This file is part of a c++ utility library for platforms where no
 * full c++ standard library is available.
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
 * @file    utils_avr.h
 * @brief   Helper functions for AVR programming
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#ifndef SRC_UTILS_H_
#define SRC_UTILS_H_

#include <cstdint>
#include <ostream>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/builtins.h>


namespace avr {

/**
 * @brief Helper type for storing C-strings in program memory on AVR
 * @note Based on Arduino implementation <https://github.com/arduino/ArduinoCore-avr>
 */
class FlashStringHelper;
#define F(string_literal) (reinterpret_cast<const avr::FlashStringHelper*>(PSTR(string_literal)))

inline std::ostream& operator <<(std::ostream& os, const avr::FlashStringHelper* v) {
    auto ptr(reinterpret_cast<PGM_P>(v));
    while (true) {
        const uint8_t c { pgm_read_byte(ptr++) };
        if (c == 0) {
            break;
        }
        os << c;
    }

    return os;
}

/**
 * @brief Convert an address given as integer into a pointer type T*
 * @tparam T: Type of Pointer
 * @param[in] ptr: Address the returned pointer shall point to
 * @return Pointer of Type T* to Address ptr
 */
template <typename T>
constexpr auto PTR(const intptr_t ptr) {
    return reinterpret_cast<volatile T*>(ptr);
}

/**
 * @brief Convert an address given as integer into a pointer of type uint8_t*
 * @param[in] ptr: Address the returned pointer shall point to
 * @return Pointer of uint8_t* to Address ptr
 */
constexpr auto PTR_8(const intptr_t ptr) {
    return PTR<uint8_t>(ptr);
}

/**
 * @brief Convert an address given as integer into a pointer of type uint16_t*
 * @param[in] ptr: Address the returned pointer shall point to
 * @return Pointer of uint16_t* to Address ptr
 */
constexpr auto PTR_16(const intptr_t ptr) {
    return PTR<uint16_t>(ptr);
}

/**
 * @brief Bit shift 1 by n bits
 * @tparam T: Type of return value
 * @param[in] bit: Number of bits to shift
 * @return 1 << bit with type T
 */
template <typename T>
inline static constexpr T BV(const uint8_t bit) {
    return static_cast<T>(1U << bit);
}

/**
 * @brief Bit shift 1 by n bits
 * @param[in] bit: Number of bits to shift
 * @return 1 << bit as uint8_t
 */
inline static constexpr uint8_t BV_8(const uint8_t bit) {
    return BV<uint8_t>(bit);
}

/**
 * @brief Bit shift 1 by n bits
 * @param[in] bit: Number of bits to shift
 * @return 1 << bit as uint16_t
 */
inline static constexpr uint16_t BV_16(const uint8_t bit) {
    return BV<uint16_t>(bit);
}


/**
 * @brief
 * @tparam T:
 * @param[in] sfr:
 * @param[in] bit:
 * @return
 */
template <typename T>
inline static void CBI(volatile T* sfr, const uint8_t bit) {
    const T mask { BV<T>(bit) };
    *sfr &= ~mask;
}

/**
 * @brief
 * @tparam T:
 * @param[in] sfr:
 * @param[in] bit:
 */
template <typename T>
inline static void CBI(intptr_t sfr, const uint8_t bit) {
    auto ptr(PTR<T>(sfr));
    const T mask { BV<T>(bit) };
    *ptr &= ~mask;
}

/**
 * @brief
 * @tparam T:
 * @param[in] sfr:
 * @param[in] bit:
 */
template <typename T>
inline static void SBI(volatile T* sfr, const uint8_t bit) {
    const T mask { BV<T>(bit) };
    *sfr |= mask;
}

/**
 * @brief
 * @tparam T:
 * @param[in] sfr:
 * @param[in] bit:
 */
template <typename T>
inline static void SBI(intptr_t sfr, const uint8_t bit) {
    auto ptr(PTR<T>(sfr));
    const T mask { BV<T>(bit) };
    *ptr |= mask;
}


/**
 * @brief
 * @tparam ACTIVE:
 */
template <bool ACTIVE = true>
class IntLock {
private:
    uint8_t sreg_;

public:
    IntLock();

    ~IntLock() {
        if (ACTIVE) {
            SREG = sreg_;
        }
    }

    void operator =(const IntLock&) = delete;
};

/**
 * @brief Specialization of constructor for ACTIVE == false
 * @note Does nothing
 */
template<>
inline IntLock<false>::IntLock() {}

/**
 * @brief Specialization of constructor for ACTIVE == true
 * @note Clears global interrupt flag
 */
template<>
inline IntLock<true>::IntLock() : sreg_ { SREG } {
    __builtin_avr_cli();
}


/**
 * @brief Helper class to execute a piece of code (functor / lambda) atomically
 * @tparam FROM_ISR: Set to true, if used in an ISR; false per default
 */
template <bool FROM_ISR = false>
struct ExecuteAtomic {
    /**
     * @brief Call operator
     * @tparam F: Type of Functor to be executed atomically
     * @param[in] func: Functor (or lambda) which implements the block to be executed atomically
     * @return Execution of func()
     * @see IntLock
     */
    template <typename F>
    auto operator()(F func) const -> decltype(func()) {
        IntLock<! FROM_ISR> lock;
        return func();
    }
};

} /* namespace avr */

#endif /* SRC_UTILS_H_ */
