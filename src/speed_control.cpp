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
 * @file    speed_control.h
 * @brief   Motor speed controller
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#include "speed_control.h"
#include "ctbot.h"
#include "scheduler.h"

#include <PID_v1.h>
#include <cmath>


namespace ctbot {

std::list<SpeedControl*> SpeedControl::controller_list_;

SpeedControl::SpeedControl(Encoder& wheel_enc, Motor& motor) : direction_ { true }, setpoint_ { 0.f }, kp_ { 40 }, ki_ { 30 }, kd_ { 0 },
        p_pid_controller_ { new Pid(input_, output_, setpoint_, kp_, ki_, kd_, true) }, wheel_encoder_ { wheel_enc }, motor_ { motor } {
    if (! p_pid_controller_) {
        return;
    }

    p_pid_controller_->set_sample_time(20);
    p_pid_controller_->set_output_limits(0, CtBotConfig::MOT_PWM_MAX);
    p_pid_controller_->set_mode(Pid::Modes::AUTOMATIC);

    controller_list_.push_back(this);

    if (controller_list_.size() == 1) {
        CtBot::get_instance().get_scheduler()->task_add("sctrl", TASK_PERIOD_MS, &controller, nullptr);
    }
}

SpeedControl::~SpeedControl() {
    controller_list_.remove(this);
    delete p_pid_controller_;
}

void SpeedControl::run() {
    if (! p_pid_controller_) {
        return;
    }

    input_ = std::fabs(get_enc_speed());
    const auto now_ms(Timer::get_ms());
    p_pid_controller_->compute(now_ms);

    int16_t pwm;
    if (setpoint_ == 0.f) {
        pwm = 0;
    } else {
        pwm = static_cast<int16_t>(output_);
    }

    motor_.set(direction_ ? pwm : static_cast<int16_t>(-pwm));
}

void SpeedControl::set_parameters(const float kp, const float ki, const float kd) {
    if (! p_pid_controller_) {
        return;
    }

    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    p_pid_controller_->set_tunings(kp_, ki_, kd_);
}

void SpeedControl::controller(void*) {
    // std::cout << "SpeedControl::controller(): running speed controller at " << Timer::get_ms() << " ms\n";

    for (auto p_ctrl : controller_list_) {
        p_ctrl->run();
    }
}

} /* namespace ctbot */
