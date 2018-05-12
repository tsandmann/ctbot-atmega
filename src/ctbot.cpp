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
 * @file    ctbot.cpp
 * @brief   Main class of c't-Bot ATmega framework
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#include "ctbot.h"
#include "ctbot_config.h"
#include "scheduler.h"
#include "leds.h"
#include "display.h"
#include "ena.h"
#include "sensors.h"
#include "motor.h"
#include "speed_control.h"
#include "servo.h"
#include "serial_connection_avr.h"
#include "cmd_parser.h"

#include <serial_port.h>
#include <cstdint>
#include <iostream>
#include <avr/sleep.h>
#include <util/delay.h>


namespace ctbot {

const char CtBot::usage_text[] {
    "command\tsubcommand [param]\texplanation\n"
    "---------------------------------------------------\n"
    "help (h)\t\t\tprint this help message\n"
    "halt\t\t\t\tshutdown and put ATmega in sleep mode\n"
    "\n"

    "config (c)\n"
    "\techo [0|1]\t\tset console echo on/off\n"
    "\ttask ledtest [0|1]\t\tstart/stop LED test\n"
    "\ttask sctrl [0|1]\t\tstart/stop speed controller task\n"
    "\ttask [taskname] [0|1]\t\tstart/stop a task\n"
    "\tk{p,i,d} [0;65535]\tset Kp/Ki/Kd parameter for speed controller\n"
    "\n"

    "get (g)\n"
    "\tdist\t\t\tprint current distance sensor's values\n"
    "\tenc\t\t\tprint current encoder's values\n"
    "\tborder\t\t\tprint current border sensor's values\n"
    "\tline\t\t\tprint current line sensor's values\n"
    "\tldr\t\t\tprint current LDR sensor's values\n"

    "\tspeed\t\t\tprint current speed for left and right wheel\n"
    "\tmotor\t\t\tprint pwm for left and right motor\n"
    "\tservo\t\t\tprint setpoints for servos\n"
    "\trc5\t\t\tprint last received RC5 data\n"

    "\ttrans\t\t\tprint current transport pocket status\n"
    "\tdoor\t\t\tprint current door status\n"
    "\terror\t\t\tprint current error status\n"
    "\tled\t\t\tprint current LED setting\n"

    "\ttasks\t\t\tprint task list\n"
    "\n"

    "set (s)\n"
    "\tspeed [-100;100] [=]\t\tset new speed in % for left and right motor\n"
    "\tmotor [-16000;16000] [=]\tset new pwm for left and right motor\n"
    "\tservo [0;255] [=]\t\tset new position for servo 1 and 2\n"
    "\tled [0;255]\t\t\tset new LED setting\n"
    "\tlcd [1;4] [1;20] TEXT\t\tprint TEXT on LCD at line and column\n"
};

CtBot::CtBot() : p_serial_port_(Uart0::get_impl()) {
    /* initialize debug and command serial line */
    p_serial_port_->begin(CtBotConfig::UART0_BAUDRATE);

    std::cout << "\n\nCtBot instance created.\n\n";
}

void CtBot::start() {
    p_scheduler_->print_task_list(std::cout);

    p_scheduler_->run();

    shutdown();
}

void CtBot::stop() {
    if (p_scheduler_) {
        p_scheduler_->stop();
    }
}

void CtBot::setup() {
    Timer::init();
    p_scheduler_ = new Scheduler();
    p_scheduler_->task_add("main", TASK_PERIOD_MS, [] (void* p_data) { auto p_this(reinterpret_cast<CtBot*>(p_data)); return p_this->run(); }, this);

    p_sensors_ = new Sensors();

    p_motors_[0] = new Motor(p_sensors_->get_enc_l(), PTR_8(CtBotConfig::MOT_L_PWM_REG::DDR), CtBotConfig::MOT_L_PWM_PIN, PTR_8(CtBotConfig::MOT_L_DIR_REG::PORT),
        PTR_8(CtBotConfig::MOT_L_DIR_REG::DDR), CtBotConfig::MOT_L_DIR_PIN, false, PTR_16(CtBotConfig::MOT_L_OCR));
    p_motors_[1] = new Motor(p_sensors_->get_enc_r(), PTR_8(CtBotConfig::MOT_R_PWM_REG::DDR), CtBotConfig::MOT_R_PWM_PIN, PTR_8(CtBotConfig::MOT_R_DIR_REG::PORT),
        PTR_8(CtBotConfig::MOT_R_DIR_REG::DDR), CtBotConfig::MOT_R_DIR_PIN, true, PTR_16(CtBotConfig::MOT_R_OCR));

    p_speedcontrols_[0] = new SpeedControl(p_sensors_->get_enc_l(), *p_motors_[0]);
    p_speedcontrols_[1] = new SpeedControl(p_sensors_->get_enc_r(), *p_motors_[1]);

    p_servos_[0] = new Servo(Servo::ID::SERVO_1);
    p_servos_[1] = new Servo(Servo::ID::SERVO_2);

    p_leds_ = new Leds();
    p_lcd_ = new Display();

    p_parser_ = new CmdParser();
    p_serial_ = new SerialConnectionAVR(*p_serial_port_);
    p_comm_ = new CommInterfaceCmdParser(*p_serial_, *p_parser_, true);

    __builtin_avr_sei(); // FIXME: abstraction

    init_parser();

    p_comm_->debug_print(F("\n*** c't-Bot init done. ***\n\n"));
}

void CtBot::init_parser() {
    p_parser_->register_cmd(F("help"), 'h', [] (const std::string&) {
        CtBot::get_instance().p_comm_->debug_print(reinterpret_cast<const avr::FlashStringHelper*>(usage_text));
        return true;
    });

    p_parser_->register_cmd(F("halt"), [] (const std::string&) {
        CtBot::get_instance().stop();
        return true;
    });

    p_parser_->register_cmd(F("config"), 'c', [] (const std::string& args) {
        auto const p_this(&CtBot::get_instance());

        if (args.find("echo") != args.npos) {
            uint8_t v;
            CmdParser::split_args(args, v);
            p_this->p_comm_->set_echo(v);
        } else if (args.find("task") != args.npos) {
            const auto s(args.find(" ") + 1);
            const auto e(args.find(" ", s));
            const std::string taskname { args.substr(s, e - s) };
            const auto task_id(p_this->get_scheduler()->task_get(taskname));

            if (task_id < 0xffff) {
                uint8_t v;
                CmdParser::split_args(args.substr(s), v);
                if (! v) {
                    p_this->get_scheduler()->task_suspend(task_id);
                } else {
                    p_this->get_scheduler()->task_resume(task_id);
                }
            } else {
                return false;
            }
        } else if (args.find("kp") != args.npos) {
            int16_t left, right;
            CmdParser::split_args(args, left, right);
            p_this->p_speedcontrols_[0]->set_parameters(static_cast<float>(left), p_this->p_speedcontrols_[0]->get_ki(), p_this->p_speedcontrols_[0]->get_kd());
            p_this->p_speedcontrols_[1]->set_parameters(static_cast<float>(right), p_this->p_speedcontrols_[1]->get_ki(), p_this->p_speedcontrols_[1]->get_kd());
        } else if (args.find("ki") != args.npos) {
            int16_t left, right;
            CmdParser::split_args(args, left, right);
            p_this->p_speedcontrols_[0]->set_parameters(p_this->p_speedcontrols_[0]->get_kp(), static_cast<float>(left), p_this->p_speedcontrols_[0]->get_kd());
            p_this->p_speedcontrols_[1]->set_parameters(p_this->p_speedcontrols_[1]->get_kp(), static_cast<float>(right), p_this->p_speedcontrols_[1]->get_kd());
        } else if (args.find("kd") != args.npos) {
            int16_t left, right;
            CmdParser::split_args(args, left, right);
            p_this->p_speedcontrols_[0]->set_parameters(p_this->p_speedcontrols_[0]->get_kp(), p_this->p_speedcontrols_[0]->get_ki(), static_cast<float>(left));
            p_this->p_speedcontrols_[1]->set_parameters(p_this->p_speedcontrols_[1]->get_kp(), p_this->p_speedcontrols_[1]->get_ki(), static_cast<float>(right));
        } else {
            return false;
        }
        return true;
    });

    p_parser_->register_cmd(F("get"), 'g', [] (const std::string& args) {
        auto const p_this(&CtBot::get_instance());

        if (args == "dist") {
            p_this->serial_print(p_this->p_sensors_->get_distance_l(), p_this->p_sensors_->get_distance_r());
        } else if (args == "enc") {
            p_this->serial_print(p_this->p_sensors_->get_enc_l().get(), p_this->p_sensors_->get_enc_r().get());
        } else if (args == "mouse") {
            p_this->serial_print(0, 0); // mouse sensor not implemented
        } else if (args == "border") {
            p_this->serial_print(p_this->p_sensors_->get_border_l(), p_this->p_sensors_->get_border_r());
        } else if (args == "line") {
            p_this->serial_print(p_this->p_sensors_->get_line_l(), p_this->p_sensors_->get_line_r());
        } else if (args == "ldr") {
            p_this->serial_print(p_this->p_sensors_->get_ldr_l(), p_this->p_sensors_->get_ldr_r());
        } else if (args == "speed") {
            const auto l(static_cast<int16_t>(p_this->p_sensors_->get_enc_l().get_speed()));
            const auto r(static_cast<int16_t>(p_this->p_sensors_->get_enc_r().get_speed()));
            p_this->serial_print(l, r);
        } else if (args == "motor") {
            p_this->serial_print(p_this->p_motors_[0]->get(), p_this->p_motors_[1]->get());
        } else if (args == "servo") {
            p_this->serial_print(p_this->p_servos_[0]->get(), p_this->p_servos_[1]->get());
        } else if (args == "rc5") {
            p_this->serial_print(p_this->p_sensors_->get_rc5().get_addr(), p_this->p_sensors_->get_rc5().get_cmd(), p_this->p_sensors_->get_rc5().get_toggle());
        } else if (args == "trans") {
            p_this->serial_print(p_this->p_sensors_->get_transport());
        } else if (args == "door") {
            p_this->serial_print(p_this->p_sensors_->get_shutter());
        } else if (args == "error") {
            p_this->serial_print(p_this->p_sensors_->get_error());
        } else if (args == "led") {
            p_this->p_comm_->debug_print("0x");
            p_this->serial_print_base(static_cast<uint8_t>(p_this->p_leds_->get()), PrintBase::HEX);
        } else if (args == "tasks") {
            p_this->get_scheduler()->print_task_list(std::cout);
        } else {
            return false;
        }
        p_this->p_comm_->debug_print('\n');
        return true;
    });

    p_parser_->register_cmd(F("set"), 's', [] (const std::string& args) {
        auto const p_this(&CtBot::get_instance());

        if (args.find("speed") != args.npos) {
            int16_t left, right;
            CmdParser::split_args(args, left, right);
            p_this->p_speedcontrols_[0]->set_speed(static_cast<float>(left));
            p_this->p_speedcontrols_[1]->set_speed(static_cast<float>(right));
        } else if (args.find("motor") != args.npos) {
            int16_t left, right;
            CmdParser::split_args(args, left, right);
            p_this->p_motors_[0]->set(left);
            p_this->p_motors_[1]->set(right);
        } else if (args.find("servo") != args.npos) {
            uint8_t s1, s2;
            CmdParser::split_args(args, s1, s2);
            p_this->p_servos_[0]->set(s1);
            p_this->p_servos_[1]->set(s2);
        } else if (args.find("led") != args.npos) {
            uint8_t led;
            CmdParser::split_args(args, led);
            p_this->p_leds_->set(static_cast<ctbot::LedTypes>(led));
        } else if (args.find("lcd") != args.npos) {
            uint8_t line, column;
            auto ptr(CmdParser::split_args(args, line, column));
            if (! line && ! column) {
                p_this->p_lcd_->clear();
                return true;
            }
            p_this->p_lcd_->set_cursor(line, column);
            if (args.length() <= static_cast<size_t>(ptr - args.c_str())) {
                return false;
            }
            while (*(++ptr) != 0) {
                p_this->p_lcd_->print(*ptr);
            }
        } else {
            return false;
        }
        return true;
    });
}

void CtBot::run() {
    // std::cout << "CtBot::run(): now=" << Timer::get_ms() << " ms \n";

    p_sensors_->update();
}

void CtBot::shutdown() {
    p_comm_->debug_print(F("System shutting down...\n"));

    p_speedcontrols_[0]->set_speed(0.f);
    p_speedcontrols_[1]->set_speed(0.f);
    p_motors_[0]->set(0.f);
    p_motors_[1]->set(0.f);
    p_servos_[0]->set(Servo::POS_OFF);
    p_servos_[1]->set(Servo::POS_OFF);
    p_lcd_->clear();
    p_leds_->set(LedTypes::NONE);
    p_sensors_->disable_all();
    p_serial_port_->flush();

    __builtin_avr_cli();
    do {
        _SLEEP_CONTROL_REG = (_SLEEP_CONTROL_REG & ~(BV_8(SM0) | BV_8(SM1) | BV_8(SM2))) | (SLEEP_MODE_PWR_DOWN);
    } while (0);
    sleep_enable();
    sleep_cpu();
}

} /* namespace ctbot */
