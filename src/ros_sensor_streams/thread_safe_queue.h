/**
 * This file is part of flame_ros.
 * Copyright (C) 2017 W. Nicholas Greene (wng@csail.mit.edu)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 *
 * @file thread_safe_queue.h
 * @author W. Nicholas Greene
 * @date 2017-08-21 21:31:26 (Mon)
 */

#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>

namespace ros_sensor_streams {

/**
 * \brief Template class for a thread-safe queue.
 *
 * Has a condition variable that will notify when queue is non-empty.
 * Heavily inspired from the NotifyBuffer class from LSD-SLAM.
 */
template <typename T>
class ThreadSafeQueue {
 public:
  /**
   * \brief Constructor.
   * @param max_queue_size Maximum queue size.
   */
  explicit ThreadSafeQueue(int max_queue_size = 8) :
    max_queue_size(max_queue_size),
    queue(),
    non_empty_(),
    mtx() {}

  ThreadSafeQueue(const ThreadSafeQueue& rhs) = delete;

  ThreadSafeQueue& operator=(const ThreadSafeQueue& rhs) = delete;

  virtual ~ThreadSafeQueue() {}

  /**
   * \brief Return number of items in queue.
   * @return Number of items.
   */
  int size() {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return queue.size();
  }

  /**
   * \brief Push item to the back of the queue or discard if queue is full.
   * @param new_item[in] The item to be pushed.
   * @return True if item pushed, False if queue is full.
   */
  bool push(const T& item) {
    std::unique_lock<std::recursive_mutex> lock(mtx);

    if (queue.size() == max_queue_size) {
      return false;
    }

    queue.push(item);
    lock.unlock();

    non_empty_.notify_one();

    return true;
  }

  /**
   * \brief Pop the first item.
   */
  void pop() {
    std::unique_lock<std::recursive_mutex> lock(mtx);
    non_empty_.wait(lock, [this](){ return !queue.empty(); });
    queue.pop();
    return;
  }

  /**
   * \brief Return a reference to the front item.
   * @return Reference.
   */
  const T& front() {
    std::unique_lock<std::recursive_mutex> lock(mtx);
    non_empty_.wait(lock, [this](){ return !queue.empty(); });
    return queue.front();
  }

  /**
   * \brief Return a handle to the underlying mutex.
   * @return The mutex.
   */
  inline std::recursive_mutex& mutex() { return mtx; }

  /**
   * \brief Non-empty condition variable.
   *
   * Use to signal that the queue has data.
   *
   * @return Condition variable.
   */
  inline std::condition_variable_any& non_empty() {
    return non_empty_;
  }

 private:
  int max_queue_size;
  std::queue<T> queue;

  std::condition_variable_any non_empty_;
  std::recursive_mutex mtx;
};

} // namespace ros_sensor_streams
