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
 * @file    serial_port.cpp
 * @brief   ATmega1284P serial port driver
 * @author  Timo Sandmann
 * @date    15.04.2018
 * @note    Based on Arduino implementation <https://github.com/arduino/ArduinoCore-avr>
 */

#include "serial_port.h"
#include "utils_avr.h"

#include <avr/interrupt.h>
#include <avr/builtins.h>


namespace avr {

void SerialPort::begin(const uint32_t baud) const {
    /* try u2x mode first */
    uint16_t baud_setting = (F_CPU / 4UL / baud - 1UL) / 2UL;
    UCSR0A = BV_8(U2X0);

    /* the baud_setting cannot be > 4095, so switch back to non-u2x mode if the baud rate is too low */
    if (baud_setting > 4095U) {
        UCSR0A = 0;
        baud_setting = (F_CPU / 8UL / baud - 1UL) / 2UL;
    }

    /* assign the baud_setting, a.k.a. ubrr (USART Baud Rate Register) */
    UBRR0H = baud_setting >> 8;
    UBRR0L = baud_setting & 0xff;

    /* set the data bits, parity, and stop bits */
    UCSR0C = 0x06; // 8N1

    SBI(&UCSR0B, RXEN0);
    SBI(&UCSR0B, TXEN0);
    SBI(&UCSR0B, RXCIE0);
    CBI(&UCSR0B, UDRIE0);
}

void SerialPort::end() {
    flush();

    /* disable UART0 */
    uint8_t tmp { UCSR0B };
    tmp &= ~(RXEN0 | TXEN0 | RXCIE0 | UDRIE0);
    UCSR0B = tmp;

    /* clear any received data */
    rx_buf_head_ = rx_buf_tail_;
}

void SerialPort::flush() const {
    while (bit_is_clear(UCSR0A, TXC0)) {}
    // if we get here the hardware finished tranmission (TXC is set)
}

int16_t SerialPort::read() {
    ExecuteAtomic<std::is_same<uint8_t, rx_buf_idx_t>::value> x;
    const rx_buf_idx_t head {
        x([this]() {
            return rx_buf_head_;
        })
    };

    if (head == rx_buf_tail_) {
        // if the head isn't ahead of the tail, we don't have any characters
        return -1;
    } else {
        const uint8_t c { rx_buffer_[rx_buf_tail_] };
        const rx_buf_idx_t new_tail { static_cast<rx_buf_idx_t>(static_cast<rx_buf_idx_t>(rx_buf_tail_ + 1U) % sizeof(rx_buffer_)) };

        ExecuteAtomic<std::is_same<uint8_t, rx_buf_idx_t>::value> x;
        x([this, new_tail]() {
            rx_buf_tail_ = new_tail;
        });

        return static_cast<int16_t>(c);
    }
}

int16_t SerialPort::read(void* buffer, const size_t length) {
    uint8_t* ptr { reinterpret_cast<uint8_t*>(buffer) };
    size_t n { 0 };
    while (n < length) {
        const int16_t c { read() };
        if (c < 0) {
            break;
        }
        *ptr++ = static_cast<uint8_t>(c);
        ++n;
    }
    return n;
}

int16_t SerialPort::write(uint8_t data) const {
    /* wait for empty transmit buffer */
    while (! (UCSR0A & (1 << UDRE0))) {}

    /* put data into buffer, sends the data */
    UDR0 = data;

    /* clear transmit complete flag */
    SBI(&UCSR0A, TXC0);

    return 1;
}

int16_t SerialPort::write(const void* buffer, const size_t length) const {
    const uint8_t* ptr { reinterpret_cast<const uint8_t*>(buffer) };
    for (size_t i(0U); i < length; ++i) {
        write(*ptr++);
    }
    return length;
}

inline __attribute__((always_inline)) void SerialPort::isr() {
    if (bit_is_clear(UCSR0A, UPE0)) {
        // no parity error, read byte and store it in the buffer
        const uint8_t c { UDR0 };
        const rx_buf_idx_t i { static_cast<rx_buf_idx_t>(static_cast<rx_buf_idx_t>(rx_buf_head_ + 1U) % sizeof(rx_buffer_)) };

        // if we should be storing the received character into the location
        // just before the tail (meaning that the head would advance to the
        // current location of the tail), we're about to overflow the buffer
        // and so we don't write the character or advance the head.
        if (i != rx_buf_tail_) {
            rx_buffer_[rx_buf_head_] = c;
            rx_buf_head_ = i;
        }
    } else {
        // parity error, read byte but discard it
        UDR0;
    };
}


SerialPort Uart0::port_ {};


ISR(USART0_RX_vect) {
    Uart0::get_impl()->isr();
}

} /* namespace avr */
