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

/**
 * @brief Cooperative scheduler implementation for periodic tasks
 */
class Scheduler {
public:
    using task_func_data_t = void*;
    using task_func_t = void (*)(task_func_data_t);

protected:
    /**
     * @brief Task control block
     */
    struct Task {
        uint16_t id_; /**< ID of task */
        uint16_t period_; /**< Execution period of task in ms */
        uint32_t next_runtime_; /**< Next runtime of task in ms */
        bool active_; /**< Flag indicating if task is runnable (true) or blocked (false) */
        task_func_t func_; /**< Function pointer to the task's implementation */
        task_func_data_t func_data_; /**< Pointer to additional data for the task */

        /**
         * @brief Construct a new Task object
         * @param[in] id: ID of task
         * @param[in] period: Execution period of task in ms
         * @param[in] func: Function pointer to the task's implementation
         * @param[in] func_data: Pointer to additional data for the task
         */
        Task(const uint16_t id, const uint16_t period, task_func_t&& func, task_func_data_t&& func_data);

        /**
         * @brief Construct a new Task object and set next runtime
         * @param[in] id: ID of task
         * @param[in] period: Execution period of task in ms
         * @param[in] next_run: Next runtime of task in ms
         * @param[in] func: Function pointer to the task's implementation
         * @param[in] func_data: Pointer to additional data for the task
         */
        Task(const uint16_t id, const uint16_t period, const uint32_t next_run, task_func_t&& func, task_func_data_t&& func_data);

        /**
         * @brief Compare less operator
         * @param[in] other: Reference to task to compare with
         * @return true, if the other tasks has to be executed before (a lower next runtime)
         */
        bool operator <(const Task& other) const {
            return next_runtime_ > other.next_runtime_; // lowest next_runtime will be executed first!
        }
    };

    volatile bool running_; /**< Flag indicating the current status of the scheduler (running, iff true) */
    uint16_t next_id_; /**< Next task ID to use */
    std::priority_queue<Task> task_queue_; /**< Queue of all tasks, sorted ascending by next runtime */
    std::vector<Task>& task_vector_; /**< Reference to underlying container of task_queue_ */
    std::vector<std::string> task_names_; /**< Vector containing the tasks' names, ordering by task IDs */

    /**
     * @brief Stream operator to print task information to an output stream
     * @param[out] os: Reference to output stream to print information to
     * @param[in] v: Reference to task which information shall be printed
     * @return Reference to used output stream
     */
    friend std::ostream& operator <<(std::ostream& os, const Task& v);

    /**
     * @brief Helper function to get access to the underlying container of a std::priority_queue
     * @tparam T: The type of the stored elements (as for std::priority_queue)
     * @tparam S: The type of the underlying container to use to store the elements (as for std::priority_queue)
     * @tparam C: A Compare type providing a strict weak ordering (as for std::priority_queue)
     * @param[in] q: Reference to a std::priority_queue we want to get the underlying container of type S
     * @return Reference to the underlying container of type S
     * @note See <https://stackoverflow.com/a/1385520>
     */
    template <class T, class S, class C>
    S& Container(std::priority_queue<T, S, C>& q) {
        struct HackedQueue : private std::priority_queue<T, S, C> {
            static S& Container(std::priority_queue<T, S, C>& q) {
                return q.*&HackedQueue::c;
            }
        };
        return HackedQueue::Container(q);
    }

public:
    /**
     * @brief Construct a new Scheduler object
     */
    Scheduler() : running_ { true }, next_id_ { 0 }, task_vector_ { Container(task_queue_) } {}

    /**
     * @brief Add a tasks to the run queue
     * @param[in] name: Reference to string with name of the task
     * @param[in] period: Execution period of task in ms
     * @param[in] func: Function pointer to the task's implementation
     * @param[in] func_data: Pointer to additional data for the task
     * @return ID of created task
     */
    uint16_t task_add(const std::string& name, const uint16_t period, task_func_t&& func, task_func_data_t&& func_data);

    /**
     * @brief Get the ID of a task given by its name
     * @param[in] name: Reference to string with task name
     * @return ID of searched task or 0xffff, if task name is unknown
     */
    uint16_t task_get(const std::string& name) const;

    /**
     * @brief Suspend a task, task will not be scheduled again until resumed
     * @param[in] id: ID of task to suspend
     * @return true on success
     */
    bool task_suspend(const uint16_t id);

    /**
     * @brief Resume a task, task will be scheduled on next runtime
     * @param[in] id: ID of task to resume
     * @return true on success
     */
    bool task_resume(const uint16_t id);

    /**
     * @brief Entry point of scheduler
     * @note Does not return until scheduler is stoppped
     */
    void run();

    /**
     * @brief Stop the scheduler (and its main loop, @see run)
     */
    void stop() {
        running_ = false;
    }

    /**
     * @brief Print a list of all tasks and their current status
     * @param[out] os: Reference to output stream, where the list will be printed to
     */
    void print_task_list(std::ostream& os) const;
};

} /* namespace ctbot */

#endif /* SRC_SCHEDULER_H_ */
