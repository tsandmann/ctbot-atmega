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
 * @file    ctbot_main.cpp
 * @brief   Entry point of c't-Bot atmega framework
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#include "ctbot.h"
#include "scheduler.h"
#include "ctbot_config.h"
#include "leds.h"
#include "tests.h"

#include <avr/builtins.h>
#include <util/delay.h>


/**
 * @brief Main function as entry point for c't-Bot initialization
 *
 * @startuml{main.png}
 *  activate Main
 *  Main -> CtBot: get_instance()
 *  Main <-- CtBot: ctbot
 *
 *  Main -> CtBot: setup()
 *  activate CtBot
 *  create Scheduler
 *  CtBot -> Scheduler: new
 *  CtBot -> Scheduler: task_add("main")
 *  activate Scheduler
 *  CtBot <-- Scheduler
 *  deactivate Scheduler
 *
 *  Main <-- CtBot
 *  deactivate CtBot
 *
 *  alt LED_TEST_AVAILABLE == true
 *   create LedTest
 *   Main -> LedTest: new
 *   LedTest -> Scheduler: task_add("ledtest")
 *   activate Scheduler
 *   LedTest <-- Scheduler
 *   deactivate Scheduler
 *   activate LedTest
 *   Main <-- LedTest
 *  end
 *
 *  alt LCD_TEST_AVAILABLE == true
 *   create LcdTest
 *   Main -> LcdTest: new
 *   LcdTest -> Scheduler: task_add("lcdtest")
 *   activate Scheduler
 *   LcdTest <-- Scheduler
 *   deactivate Scheduler
 *   activate LcdTest
 *   Main <-- LcdTest
 *  end
 *
 *  alt ENA_TEST_AVAILABLE == true
 *   create EnaTest
 *   Main -> EnaTest: new
 *   EnaTest -> Scheduler: task_add("enatest")
 *   activate Scheduler
 *   EnaTest <-- Scheduler
 *   deactivate Scheduler
 *   activate EnaTest
 *   Main <-- EnaTest
 *  end
 *
 *  alt SENS_LCD_TEST_AVAILABLE == true
 *   create SensorLcdTest
 *   Main -> SensorLcdTest: new
 *   SensorLcdTest -> Scheduler: task_add("senstest")
 *   activate Scheduler
 *   SensorLcdTest <-- Scheduler
 *   deactivate Scheduler
 *   activate SensorLcdTest
 *   Main <-- SensorLcdTest
 *  end
 *
 *  Main -> CtBot: start()
 *  activate CtBot
 *  ... **run until shutdown** ...
 *  Main <-- CtBot
 *  deactivate SensorLcdTest
 *  destroy SensorLcdTest
 *  deactivate EnaTest
 *  destroy EnaTest
 *  deactivate LcdTest
 *  destroy LcdTest
 *  deactivate LedTest
 *  destroy LedTest
 *  note over Scheduler: tasks aren't really destroyed on shutdown
 *  deactivate CtBot
 * @enduml
 */
int main() {
    /* disable interrupts */
    __builtin_avr_cli();
    _delay_ms(100.f);

    /* create CtBot singleton instance... */
    auto& ctbot(ctbot::CtBot::get_instance());
    /* ..., initialize it... */
    ctbot.setup();

    /* create test tasks if configured */
    if (ctbot::CtBotConfig::LED_TEST_AVAILABLE) {
        new ctbot::tests::LedTest(ctbot);
    }

    if (ctbot::CtBotConfig::LCD_TEST_AVAILABLE) {
        new ctbot::tests::LcdTest(ctbot);
    }

    if (ctbot::CtBotConfig::ENA_TEST_AVAILABLE) {
        new ctbot::tests::EnaTest(ctbot);
    }

    if (ctbot::CtBotConfig::SENS_LCD_TEST_AVAILABLE) {
        new ctbot::tests::SensorLcdTest(ctbot);
    }

    /* ... and start CtBot instance */
    ctbot.start();
}
