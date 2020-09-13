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
 * @file    scheduler.h
 * @brief   Task scheduling
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#include "scheduler.h"
#include "timer.h"

#include <utils_avr.h>
#include <iostream>
#include <avr/sleep.h>


namespace ctbot {
using namespace avr;

Scheduler::Task::Task(const uint16_t id, const uint16_t period, task_func_t&& func, task_func_data_t&& func_data)
    : id_ { id }, period_ { static_cast<uint16_t>(Timer::ms_to_ticks(period)) }, next_runtime_ { 0 }, active_ { true }, func_ { std::move(func) }, func_data_ {
          std::move(func_data)
      } {}

Scheduler::Task::Task(const uint16_t id, const uint16_t period, const uint32_t next_run, task_func_t&& func, task_func_data_t&& func_data)
    : id_ { id }, period_ { static_cast<uint16_t>(Timer::ms_to_ticks(period)) }, next_runtime_ { next_run }, active_ { true }, func_ { std::move(func) },
      func_data_ { std::move(func_data) } {}

std::ostream& operator<<(std::ostream& os, const Scheduler::Task& v) {
    os << std::hex << "0x" << v.id_ << "\t" << (v.active_ ? F("  ACTIVE") : F("INACTIVE")) << "\t" << std::dec << Timer::ticks_to_us(v.period_) << " us\n";
    return os;
}

void Scheduler::run() {
    while (running_ && (!task_queue_.empty())) {
        const Task& task { task_queue_.top() };
        // std::cout << "Scheduler::run(): task.next_runtime=" << task.next_runtime_ << " ticks.\n";
        const uint32_t next_runtime { task.next_runtime_ };
        uint32_t now { Timer::get_tickcount() };
        // std::cout << "Scheduler::run(): now=" << now << " ticks \n";
        while (next_runtime > now) {
            do {
                _SLEEP_CONTROL_REG = static_cast<uint8_t>((_SLEEP_CONTROL_REG & ~(BV_8(SM0) | BV_8(SM1) | BV_8(SM2))) | (SLEEP_MODE_IDLE));
            } while (0);
            sleep_enable();
            sleep_cpu();
            now = Timer::get_tickcount();
        }
        // std::cout << "Scheduler::run(): now=" << now << " ticks \n";
        if (task.active_) {
            // std::cout << "Scheduler::run(): executing function with period " << Timer::ticks_to_us(task.period_) << " us.\n";
            task.func_(task.func_data_);
            // std::cout << "Scheduler::run(): function with period " << Timer::ticks_to_us( task.period_) << " us done.\n";
        }
        Task new_task { task };
        new_task.next_runtime_ = now + new_task.period_;
        task_queue_.pop();
        task_queue_.push(new_task);
    }
}

uint16_t Scheduler::task_add(const std::string& name, const uint16_t period, task_func_t&& func, task_func_data_t&& func_data) {
    Task task { next_id_, period, Timer::get_tickcount(), std::move(func), std::move(func_data) };
    task_queue_.push(task);
    task_names_.push_back(name);

    std::cout << F("Scheduler::task_add(): task \"") << task_names_[next_id_] << F("\" with id 0x") << std::hex << task.id_ << F(" and period of ") << std::dec
              << Timer::ticks_to_us(task.period_) << F(" us added.\n");
    return next_id_++;
}

uint16_t Scheduler::task_get(const std::string& name) const {
    for (uint16_t i { 0U }; i < next_id_; ++i) {
        if (task_names_[i] == name) {
            return i;
        }
    }

    return 0xffff;
}

bool Scheduler::task_suspend(const uint16_t id) {
    // std::cout << "Scheduler::task_suspend(0x" << std::hex << id << std::dec << ")\n";
    bool res { false };
    for (auto& t : task_vector_) {
        if (t.id_ == id) {
            t.active_ = false;
            res = true;
            break;
        }
    }

    return res;
}

bool Scheduler::task_resume(const uint16_t id) {
    // std::cout << "Scheduler::task_resume(0x" << std::hex << id << std::dec << ")\n";
    bool res { false };
    for (auto& t : task_vector_) {
        if (t.id_ == id) {
            t.active_ = true;
            res = true;
            break;
        }
    }

    return res;
}

void Scheduler::print_task_list(std::ostream& os) const {
    for (uint16_t i { 0U }; i < next_id_; ++i) {
        os << " \"" << task_names_[task_vector_[i].id_] << "\":\t" << task_vector_[i];
    }
}

} /* namespace ctbot */
