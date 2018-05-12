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

#ifndef SRC_SPEED_CONTROL_H_
#define SRC_SPEED_CONTROL_H_

#include "encoder.h"
#include "motor.h"

#include <cstdint>
#include <list>


class Pid;

namespace ctbot {

/**
 * @brief Motor speed controller
 */
class SpeedControl {
protected:
    static constexpr uint16_t MAX_SPEED { 450 }; /**< Maximum possible speed in mm/s */
    static constexpr uint16_t TASK_PERIOD_MS { 1U }; /**< Scheduling period of task in ms */

    static std::list<SpeedControl*> controller_list_; /**< List of all SpeedControl instances created */

    bool direction_;
    float setpoint_, input_, output_;
    float kp_, ki_, kd_;
    Pid* p_pid_controller_;
    Encoder& wheel_encoder_;
    Motor& motor_;

    /**
     * @brief Perform the PID controller update step
     * @note Is called periodically by the speed controller task every TASK_PERIOD_MS ms
     */
    void run();

    /**
     * @brief Speed controller task implementation
     * @note Calls run() on all SpeedControl instances
     */
    static void controller(void*);

public:
    /**
     * @brief Construct a new SpeedControl object
     * @param[in] wheel_enc: Reference to wheel encoder to use for controller input (speed)
     * @param[in] motor: Reference to motor driver to use for controller output (PWM duty cycle)
     */
    SpeedControl(Encoder& wheel_enc, Motor& motor);

    /**
     * @brief Destroy the Speed Control object
     */
    ~SpeedControl();

    /**
     * @brief Set a new desired speed
     * @param[in] speed: Speed to set as percentage value; [-100; +100]
     */
    void set_speed(const float speed) {
        auto abs_speed(fabs(speed));
        if (abs_speed > 100.f) {
            abs_speed = 100.f;
        }
        setpoint_ = abs_speed * (MAX_SPEED / 100.f); // speed is in %
        direction_ = speed >= 0.f;
    }

    /**
     * @return Current speed set
     */
    auto get_speed() const {
        return setpoint_ / (direction_ ? MAX_SPEED / 100.f : MAX_SPEED / -100.f); // convert speed to %
    }

    /**
     * @return Current wheel speed as reported by related wheel encoder
     */
    auto get_enc_speed() const {
        return wheel_encoder_.get_speed();
    }

    /**
     * @return Current Kp parameter setting
     */
    auto get_kp() const {
        return kp_;
    }

    /**
     * @return Current Ki parameter setting
     */
    auto get_ki() const {
        return ki_;
    }

    /**
     * @return Current Kd parameter setting
     */
    auto get_kd() const {
        return kd_;
    }

    /**
     * @brief Set new parameters for underlying PID controller
     * @param[in] kp: Proportional tuning parameter
     * @param[in] ki: Integral tuning parameter
     * @param[in] kd: Derivative tuning parameter
     */
    void set_parameters(const float kp, const float ki, const float kd);
};

} /* namespace ctbot */

#endif /* SRC_SPEED_CONTROL_H_ */
