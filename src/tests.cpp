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
 * @file    tests.cpp
 * @brief   Test classes implementing tasks to test functionality
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#include "tests.h"
#include "ctbot.h"
#include "scheduler.h"
#include "leds.h"
#include "display.h"
#include "ena.h"
#include "sensors.h"

#include <iostream>


namespace ctbot {
namespace tests {

LedTest::LedTest(CtBot& ctbot) : ctbot_(ctbot) {
    ctbot_.get_scheduler()->task_add("ledtest", TASK_PERIOD_MS, [] (void* p_data) { auto p_this(reinterpret_cast<LedTest*>(p_data)); return p_this->run(); }, this);
}

void LedTest::run() {
    static uint8_t led_idx { 7 };

    LedTypes led(static_cast<LedTypes>(1 << led_idx));
    ctbot_.get_leds()->off(led);

    if (++led_idx > 7) {
        led_idx = 0;
    }

    led = static_cast<LedTypes>(1 << led_idx);
    ctbot_.get_leds()->on(led);
}


LcdTest::LcdTest(CtBot& ctbot) : ctbot_(ctbot) {
    ctbot_.get_scheduler()->task_add("lcdtest", TASK_PERIOD_MS, [] (void* p_data) { auto p_this(reinterpret_cast<LcdTest*>(p_data)); return p_this->run(); }, this);
}

void LcdTest::run() {
    static uint32_t x { 0UL };
    ctbot_.get_lcd()->set_cursor((x % 80) / 20 + 1, x % 20 + 1);
    ctbot_.get_lcd()->print(' ');
    ++x;
    ctbot_.get_lcd()->set_cursor((x % 80) / 20 + 1, x % 20 + 1);
    ctbot_.get_lcd()->print('*');
    ctbot_.get_lcd()->set_cursor(((x + 20) % 80) / 20 + 1, 1);
    ctbot_.get_lcd()->print(F("Hello World :-)"));
    ctbot_.get_lcd()->set_cursor(((x + 40) % 80) / 20 + 1, 1);
    ctbot_.get_lcd()->printf(F("%5u"), static_cast<uint16_t>(x));
}


EnaTest::EnaTest(CtBot& ctbot) {
    ctbot.get_scheduler()->task_add("enatest", TASK_PERIOD_MS, [] (void* p_data) { auto p_this(reinterpret_cast<EnaTest*>(p_data)); return p_this->run(); }, this);
}

void EnaTest::run() {
    static Ena* p_ena { new Ena };
    static uint8_t ena_idx { 0 };

    p_ena->set(static_cast<EnaTypes>(1 << ena_idx++));
    if (ena_idx > 7) {
        ena_idx = 0;
    }
}


SensorLcdTest::SensorLcdTest(CtBot& ctbot) : ctbot_(ctbot) {
    ctbot.get_scheduler()->task_add("senstest", TASK_PERIOD_MS, [] (void* p_data) { auto p_this(reinterpret_cast<SensorLcdTest*>(p_data)); return p_this->run(); }, this);
}

void SensorLcdTest::run() {
    auto const p_sens(ctbot_.get_sensors());

    ctbot_.get_lcd()->set_cursor(1, 1);
    ctbot_.get_lcd()->printf(F("P%03X %03X D=%4d %4d"), p_sens->get_ldr_l(), p_sens->get_ldr_r(), p_sens->get_distance_l(), p_sens->get_distance_r());

    ctbot_.get_lcd()->set_cursor(2, 1);
    ctbot_.get_lcd()->printf(F("B=%03X %03X L=%03X %03X "), p_sens->get_border_l(), p_sens->get_border_r(), p_sens->get_line_l(), p_sens->get_line_r());

    ctbot_.get_lcd()->set_cursor(3, 1);
    ctbot_.get_lcd()->printf(F("R=%2d %2d F=%d K=%d T=%d "), abs(p_sens->get_enc_l().get()) % 100, abs(p_sens->get_enc_r().get()) % 100, p_sens->get_error(), p_sens->get_shutter(), p_sens->get_transport());

    ctbot_.get_lcd()->set_cursor(4, 1);
    ctbot_.get_lcd()->printf(F("S=%4d %4d   "), static_cast<int16_t>(p_sens->get_enc_l().get_speed()), static_cast<int16_t>(p_sens->get_enc_r().get_speed()));
}

} /* namespace tests */
} /* namespace ctbot */
