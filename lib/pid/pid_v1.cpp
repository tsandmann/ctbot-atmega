/**
 * @file    PID_v1.cpp
 * @author  Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 * @author  Timo Sandmann
 * @date    04.06.2017
 * @brief   PID Library for use with ct-Bot framework
 * @version 1.2.0
 * @see     https://github.com/br3ttb/Arduino-PID-Library
 *
 * based on Arduino PID Library - Version 1.1.1 by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com, licensed under a GPLv3 License
 */

#include "pid_v1.h"


Pid::Pid(pid_t& input, pid_t& output, pid_t& setpoint, const pid_t kp, const pid_t ki, const pid_t kd, const bool direction)
    : input_ { input }, output_ { output }, setpoint_ { setpoint }, out_min_ { 0 }, out_max_ { 255 }, in_auto_ { false }, direction_ { direction },
      sample_time_ { 100 }, last_time_ { 0 } {
    set_tunings(kp, ki, kd);

    initialize();
}

bool Pid::compute(const uint32_t time_ms) {
    if (!in_auto_) {
        return false;
    }

    const auto time_change(time_ms - last_time_);

    if (time_change >= sample_time_) {
        /* Compute all the working error variables */
        const auto error(setpoint_ - input_);
        i_term_ += (ki_ * error);
        if (i_term_ > out_max_) {
            i_term_ = out_max_;
        } else if (i_term_ < out_min_) {
            i_term_ = out_min_;
        }
        const auto d_input(input_ - last_input_);

        /* Compute Pid output */
        auto output(kp_ * error + i_term_ - kd_ * d_input);

        if (output > out_max_) {
            output = out_max_;
        } else if (output < out_min_) {
            output = out_min_;
        }
        output_ = output;

        /* Remember some variables for next time */
        last_input_ = input_;
        last_time_ = time_ms;

        return true;
    } else {
        return false;
    }
}

void Pid::set_tunings(const pid_t kp, const pid_t ki, const pid_t kd) {
    if (kp < 0.f || ki < 0.f || kd < 0.f) {
        return;
    }

    disp_kp_ = kp;
    disp_ki_ = ki;
    disp_kd_ = kd;

    const pid_t sample_time_s((static_cast<pid_t>(sample_time_)) / static_cast<pid_t>(1000.));
    kp_ = kp;
    ki_ = ki * sample_time_s;
    kd_ = kd / sample_time_s;

    if (!direction_) {
        kp_ = (0.f - kp_);
        ki_ = (0.f - ki_);
        kd_ = (0.f - kd_);
    }
}

void Pid::set_sample_time(const uint16_t new_sample_time) {
    const auto ratio(static_cast<pid_t>(new_sample_time) / static_cast<pid_t>(sample_time_));
    ki_ *= ratio;
    kd_ /= ratio;
    sample_time_ = new_sample_time;
}

void Pid::set_output_limits(const pid_t min, const pid_t max) {
    if (min >= max) {
        return;
    }
    out_min_ = min;
    out_max_ = max;

    if (in_auto_) {
        if (output_ > out_max_) {
            output_ = out_max_;
        } else if (output_ < out_min_) {
            output_ = out_min_;
        }

        if (i_term_ > out_max_) {
            i_term_ = out_max_;
        } else if (i_term_ < out_min_) {
            i_term_ = out_min_;
        }
    }
}

void Pid::set_mode(const Modes new_mode) {
    const bool new_auto { new_mode == Modes::AUTOMATIC };
    if (new_auto && !in_auto_) {
        /* we just went from manual to auto mode */
        initialize();
    }
    in_auto_ = new_auto;
}

void Pid::initialize() {
    i_term_ = output_;
    last_input_ = input_;
    if (i_term_ > out_max_) {
        i_term_ = out_max_;
    } else if (i_term_ < out_min_) {
        i_term_ = out_min_;
    }
}

void Pid::set_controller_direction(const bool direction) {
    if (in_auto_ && direction != direction_) {
        kp_ = (0.f - kp_);
        ki_ = (0.f - ki_);
        kd_ = (0.f - kd_);
    }
    direction_ = direction;
}
