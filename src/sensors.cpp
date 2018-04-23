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
 * @file    sensors.h
 * @brief   c't-Bot sensor abstraction layer
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#include "sensors.h"


namespace ctbot {

Sensors::Sensors() {
    ena_.off(DigitalSensors::ENA_MASK); // disable digital sensors
    ena_.off(AnalogSensors::ENA_MASK); // disable analog sensors
    ena_.on(EnaTypes::DISTANCE); // enable distance sensors (always-on)
    ena_.on(EnaTypes::WHEEL_ENC); // enable wheel encoder (always-on)
}

void Sensors::update() {
    ena_.on(DigitalSensors::ENA_MASK | AnalogSensors::ENA_MASK);

    AnalogSensors::update();
    DigitalSensors::update();

    ena_.off(DigitalSensors::ENA_MASK | AnalogSensors::ENA_MASK);
}

} /* namespace ctbot */
