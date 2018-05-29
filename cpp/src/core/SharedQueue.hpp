/* Copyright (c) 2017-2018, CNRS-LAAS
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
#ifndef PLANNING_CPP_SHAREDQUEUE_HPP
#define PLANNING_CPP_SHAREDQUEUE_HPP

#include <condition_variable>
#include <mutex>
#include <queue>

namespace SAOP {

    template<typename T>
    class SharedQueue {
        std::queue<T> q;
        mutable std::mutex mutex;
        mutable std::condition_variable q_avail;

    public:
        SharedQueue() = default;

        SharedQueue(const SharedQueue& other) {
            std::lock_guard<std::mutex> lock(other.mutex);
            q = other.q;
        }

        // Defaults are not thread-safe
        SharedQueue& operator=(const SharedQueue& other) = delete;

        SharedQueue(SharedQueue&& other) = delete;

        SharedQueue& operator=(const SharedQueue&& other) = delete; // Default is not thread-safe

        /* Retrieve the first element
         * returns true if the queue wasn't empty and false otherwise.*/
        bool pop(T& x) {
            std::lock_guard<std::mutex> lock(mutex);
            if (q.empty()) {
                return false;
            }
            x = std::move(q.front());
            q.pop();
            return true;
        }

        /* Retrieve the first element
         * returns true if the queue wasn't empty and false otherwise.*/
        bool wait_pop(T& x) {
            std::unique_lock<std::mutex> lock(mutex);
            q_avail.wait(lock, [this] { return !q.empty(); });
            // To wait with a timeout:
            // if (!q_avail.wait_for(lock, dur, [this] { return q.empty(); })) { return false; }
            x = std::move(q.front());
            q.pop();
            return true;
        }

        /*Put an element at the end of the queue*/
        void push(T x) {
            std::lock_guard<std::mutex> lock(mutex);
            q.push(std::move(x));
            q_avail.notify_one();
        }
    };
}


#endif //PLANNING_CPP_SHAREDQUEUE_HPP
