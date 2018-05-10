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
 * @file    ctbot.h
 * @brief   Main class of c't-Bot ATmega framework
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#ifndef SRC_CTBOT_H_
#define SRC_CTBOT_H_

#include "comm_interface.h"

#include <avr/pgmspace.h>


namespace avr {
class SerialPort;
}

/**
 * @brief Namespace for all c't-Bot classes and functionality
 */
namespace ctbot {
class SerialConnectionAVR;

class Sensors;
class Motor;
class SpeedControl;
class Servo;
class Leds;
class Display;
class CmdParser;
class Scheduler;

/**
 * @brief Main class of c't-Bot ATmega framework, responsible for initialization and control loop execution
 *
 * @startuml{CtBot.png}
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 *  class ctbot::CtBot {
 *      #CtBot()
 *      #init_parser()
 *      #run()
 *      #shutdown()
 *      #serial_print<T>(data : T, base : PrintBase = PrintBase::DEC)
 *      #serial_print<T,U>(data_l : T, data_r : U, base : PrintBase = PrintBase::DEC)
 *      #serial_print<T,U,V>(data_l : T, data_r : U, data_n : V, base : PrintBase = PrintBase::DEC)
 *      +{static} get_instance() : CtBot&
 *      +~CtBot()
 *      +setup()
 *      +start()
 *      +stop()
 *      +get_sensors() : Sensors*
 *      +get_leds() : Leds*
 *      +get_lcd() : Display*
 *      +get_speedcontrols() : SpeedControl[2]*
 *      +get_servos() : Servo[2]*
 *      +get_comm() : CommInterface*
 *      +get_scheduler() : Scheduler*
 * 	    #{static} TASK_PERIOD_MS : constexpr uint16_t
 *      #{static} usage_text : const char
 *  }
 * @enduml
 */
class CtBot {
protected:
    static constexpr uint16_t TASK_PERIOD_MS { 10U }; /**< Scheduling period of task in ms */
    static const char usage_text[] PROGMEM; /**< C-String in flash memory containing the usage / help message */

    Scheduler* p_scheduler_; /**< Pointer to scheduler instance */
    Sensors* p_sensors_; /**< Pointer to sensor instance */
    Motor* p_motors_[2]; /**< Pointer to motor instances */
    SpeedControl* p_speedcontrols_[2]; /**< Pointer to speed controller instances */
    Servo* p_servos_[2]; /**< Pointer to servo instances */
    Leds* p_leds_; /**< Pointer to led instance */
    Display* p_lcd_; /**< Pointer to display instance */
    avr::SerialPort* p_serial_port_; /**< Pointer to serial port instance */
    SerialConnectionAVR* p_serial_; /**< Pointer to serial connection abstraction layer instance */
    CommInterface* p_comm_; /**< Pointer to (serial) communication interface instance */
    CmdParser* p_parser_; /**< Pointer to cmd parser instance */

    /**
     * @brief Constructor of main class
     * @note Constructor is protected to enforce singleton pattern
     *
     * The constructor initializes uart0 to allow early debug infos printed out there.
     */
    CtBot();

    /* enforce singleton */
    CtBot(const CtBot&) = delete;
    void operator=(const CtBot&) = delete;
    CtBot(CtBot&&) = delete;

    /**
     * @brief Initialize command line parser, register the actions for every command
     */
    void init_parser();

    /**
     * @brief Main task implementation
     * @note This method is run every TASK_PERIOD_MS ms, e.g. to update sensor data
     *
     * @startuml{CtBot_run.png}
     *  activate CtBot
     *  CtBot -> CtBot: setup()
     *  activate CtBot
     *  deactivate CtBot
     *
     *  loop every TASK_PERIOD_MS ms
     *    CtBot -> Sensors: update()
     *    activate Sensors
     *    CtBot <-- Sensors
     *    deactivate Sensors
     *  end
     * @enduml
     */
    void run();

    /**
     * @brief Shut everything down and put the CPU into sleep mode
     *
     * All motors, servos, leds, sensors are stopped / shut down and all tasks are suspended.
     */
    void shutdown();

    /**
     * @brief Helper function to print data to serial debug line
     * @tparam T: Type of data
     * @param[in] data: The data to print
     */
    template <typename T>
    void serial_print(T data) {
        p_comm_->debug_print(data, PrintBase::DEC);
    }

    /**
     * @brief Helper function to print data to serial debug line
     * @tparam T: Type of data
     * @tparam Args: Types of additional args
     * @param[in] data: The data to print first
     * @param[in] args: The data to print afterwards
     */
    template <typename T, typename... Args>
    void serial_print(T data, Args... args) {
        p_comm_->debug_print(data, PrintBase::DEC);
        p_comm_->debug_print(' ');
        serial_print<Args...>(args...);
    }

    /**
     * @brief Helper function to print data to serial debug line
     * @tparam T: Type of data
     * @param[in] data: The data to print
     * @param[in] base: Base of numeral system the data shall be printed in
     */
    template <typename T>
    void serial_print_base(T data, const PrintBase base = PrintBase::DEC) {
        p_comm_->debug_print(data, base);
    }

    /**
     * @brief Helper function to print data to serial debug line
     * @tparam T: Type of data
     * @tparam Args: Types of additional args
     * @param[in] data: The data to print first
     * @param[in] args: The data to print afterwards
     * @param[in] base: Base of numeral system the data shall be printed in
     */
    template <typename T, typename... Args>
    void serial_print_base(T data, Args... args, const PrintBase base = PrintBase::DEC) {
        p_comm_->debug_print(data, base);
        p_comm_->debug_print(' ');
        serial_print<Args...>(args..., base);
    }

public:
    /**
     * @brief Get the one and only instance of this class (singleton)
     * @return Reference to CtBot instance
     */
    static CtBot& get_instance() {
        static CtBot instance;
        return instance;
    }

    /**
     * @brief Default destructor
     * @note Destructor is never called in current setup
     */
    ~CtBot() = default;

    /**
     * @brief Setup method responsible for initialization and creating of instances for all components (sensors, motors, etc.)
     *
     * @startuml{CtBot_setup.png}
     *  activate CtBot
     *  CtBot -> Timer: init()
     *  activate Timer
     *  CtBot <-- Timer
     *  deactivate Timer
     *
     *  create Scheduler
     *  CtBot -> Scheduler: new
     *  CtBot <-- Scheduler: p_scheduler_
     *
     *  CtBot -> Scheduler: task_add("main")
     *  activate Scheduler
     *  CtBot <-- Scheduler
     *  deactivate Scheduler
     *
     *  create Sensors
     *  CtBot -> Sensors: new
     *  CtBot <-- Sensors: p_sensors_
     *
     *  create Motor
     *  CtBot -> Motor: new(left)
     *  CtBot <-- Motor: p_motors_[0]
     *  CtBot -> Motor: new(right)
     *  CtBot <-- Motor: p_motors_[1]
     *
     *  create SpeedControl
     *  CtBot -> SpeedControl: new(left)
     *  CtBot <-- SpeedControl: p_speedcontrols_[0]
     *  CtBot -> SpeedControl: new(right)
     *  CtBot <-- SpeedControl: p_speedcontrols_[1]
     *
     *  create Servo
     *  CtBot -> Servo: new(1)
     *  CtBot <-- Servo: p_servos_[0]
     *  CtBot -> Servo: new(2)
     *  CtBot <-- Servo: p_servos_[1]
     *
     *  create Leds
     *  CtBot -> Leds: new
     *  CtBot <-- Leds: p_leds_
     *
     *  create Display
     *  CtBot -> Display: new
     *  CtBot <-- Display: p_display_
     *
     *  create CmdParser
     *  CtBot -> CmdParser: new
     *  CtBot <-- CmdParser: p_parser_
     *
     *  create SerialConnectionAVR
     *  CtBot -> SerialConnectionAVR: new(p_serial_port_)
     *  CtBot <-- SerialConnectionAVR: p_serial_
     *
     *  create CommInterfaceCmdParser
     *  CtBot -> CommInterfaceCmdParser: new(p_serial_, p_parser_, true)
     *  CtBot <-- CommInterfaceCmdParser: p_comm_
     *
     *  CtBot -> CtBot:init_parser()
     *  CtBot -> CommInterfaceCmdParser: debug_print("c't-Bot init done.")
     *  activate CommInterfaceCmdParser
     *  CtBot <-- CommInterfaceCmdParser
     *  deactivate CommInterfaceCmdParser
     * @enduml
     */
    void setup();

    /**
     * @brief Start the scheduler
     *
     * @startuml{CtBot_start.png}
     *  activate CtBot
     *  CtBot -> Scheduler: run()
     *  deactivate CtBot
     *  activate Scheduler
     *
     *  ... run until shutdown ...
     *
     *  CtBot <-- Scheduler
     *  deactivate Scheduler
     *
     *  activate CtBot
     *  CtBot -> CtBot: shutdown()
     * @enduml
     */
    void start();

    /**
     * @brief Stop the scheduler
     *
     * @startuml{CtBot_stop.png}
     *  activate CtBot
     *  CtBot -> Scheduler: stop()
     *  activate Scheduler
     *  CtBot <-- Scheduler
     *  deactivate Scheduler
     * @enduml
     */
    void stop();

    /**
     * @brief Get the sensor instance
     * @return Pointer to sensor instance
     */
    auto get_sensors() const {
        return p_sensors_;
    }

    /**
     * @brief Get the led instance
     * @return Pointer to led instance
     */
    auto get_leds() const {
        return p_leds_;
    }

    /**
     * @brief Get the display instance
     * @return Pointer to display instance
     */
    auto get_lcd() const {
        return p_lcd_;
    }

    /**
     * @brief Get the speed controller instances of left and right motor
     * @return Pointer to speed controllers
     */
    auto get_speedcontrols() const {
        return p_speedcontrols_;
    }

    /**
     * @brief Get the servo controller intances
     * @return Pointer to servo controllers
     */
    auto get_servos() const {
        return p_servos_;
    }

    /**
     * @brief Get the communication interface instance
     * @return Pointer to communication interface instance
     */
    auto get_comm() const {
        return p_comm_;
    }

    /**
     * @brief Get the scheduler instance
     * @return Pointer to scheduler instance
     */
    auto get_scheduler() const {
        return p_scheduler_;
    }
};

} /* namespace ctbot */

#endif /* SRC_CTBOT_H_ */
