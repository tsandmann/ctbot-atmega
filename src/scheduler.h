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

#ifndef SRC_SCHEDULER_H_
#define SRC_SCHEDULER_H_

#include "ctbot_config.h"

#include <functor.h>
#include <cstdint>
#include <queue>
#include <vector>
#include <string>
#include <ostream>
#include <type_traits>


namespace ctbot {

class Scheduler {
public:
    using task_func_data_t = void*;
    using task_func_t = void (*)(task_func_data_t);

protected:
    struct Task {
        uint16_t id_;
        uint16_t period_;
        uint32_t next_runtime_;
        bool active_;
        task_func_t func_;
        task_func_data_t func_data_;

        Task(const uint16_t id, const uint16_t period, task_func_t&& func, task_func_data_t&& func_data);
        Task(const uint16_t id, const uint16_t period, const uint32_t next_run, task_func_t&& func, task_func_data_t&& func_data);

        bool operator <(const Task& b) const {
            return next_runtime_ > b.next_runtime_; // lowest next_runtime will be executed first!
        }

        friend std::ostream& operator <<(std::ostream& os, const Task& v);
    };

    volatile bool running_; /**< Flag indicating the current status of the scheduler (running, iff true) */
    uint16_t next_id_;
    std::priority_queue<Task> task_queue_;
    std::vector<Task>& task_vector_;
    std::vector<std::string> task_names_;

    friend std::ostream& operator <<(std::ostream& os, const Task& v);

    template <class T, class S, class C>
    S& Container(std::priority_queue<T, S, C> &q) {
        struct HackedQueue : private std::priority_queue<T, S, C> {
            static S& Container(std::priority_queue<T, S, C>& q) {
                return q.*&HackedQueue::c;
            }
        };
        return HackedQueue::Container(q);
    }

public:
    Scheduler() : running_ { true }, next_id_ { 0 }, task_vector_ { Container(task_queue_) } {}

    uint16_t task_add(const std::string& name, const uint16_t period, task_func_t&& func, task_func_data_t&& func_data);

    uint16_t task_get(const std::string& name) const;

    bool task_suspend(const uint16_t id);

    bool task_resume(const uint16_t id);

    void run();

    void stop() {
        running_ = false;
    }

    void print_task_list(std::ostream& os) const;
};

} /* namespace ctbot */

#endif /* SRC_SCHEDULER_H_ */
