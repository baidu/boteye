/******************************************************************************
 * Copyright 2017-2019 Baidu Robotic Vision Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#ifndef INCLUDE_DRIVER_HELPER_SYNC_SHARED_QUEUE_H_
#define INCLUDE_DRIVER_HELPER_SYNC_SHARED_QUEUE_H_

#include <driver/helper/xp_logging.h>
#include <iostream>
#include <deque>
#include <mutex>
#include <string>
#include <vector>
#include <condition_variable>

namespace XPDRIVER {

// The template class much have amethods to do synchronization comparison:
// bool T::is_older_than(const T& other)
// bool T::is_newer_than(const T& other)

template <typename T, typename SyncT = std::vector<T>, typename Container = std::deque<T> >
class SyncSharedQueue {
 public:
  // Renmove copy and assign operators
  SyncSharedQueue& operator=(const SyncSharedQueue&) = delete;
  SyncSharedQueue(const SyncSharedQueue& other) = delete;

  // capacity_ = -1 means no restriction on the sync_queue_ size
  explicit SyncSharedQueue(const int num) : num_(num), capacity_(-1), kill_(false) {
    XP_CHECK(num_ > 0);
    queues_.resize(num_);
  }
  ~SyncSharedQueue() {
    if (kill_) {
      return;
    }
    for (int i = 0; i < num_; ++i) {
      if (!queues_[i].empty()) {
        XP_LOG_ERROR("SyncSharedQueues is destructed without getting killed first");
        return;
      }
    }
  }
  void set_capacity(int capacity) {
    capacity_ = capacity;
  }

  // Use this function to kill the SyncSharedQueues before the application exits
  // to prevent potential deadlock.
  void kill() {
    kill_ = true;
    cond_.notify_all();  // Notify all for all potential subscribers to stop waiting
  }

  // Use this function to "re-initialize" the shared queue
  void reinit() {
    if (kill_) {
      kill_ = false;
      this->clear();
    }
  }

  // Operations related to individual unsync queues
  void push_back(T elem, int index) {
    bool notify = false;
    {
      std::lock_guard<std::mutex> lock(m_);
      queues_[index].push_back(std::move(elem));
      SyncT sync_elem;
      while (find_one_sync_elem(&sync_elem)) {
        if (capacity_ > 0 && sync_queue_.size() > capacity_) {
          // sync_queue is full.  Drop this sync_elem!
          continue;
        }
        sync_queue_.push_back(sync_elem);
        notify = true;
      }
    }
    // Unlock mutex m_ before notifying
    if (notify) {
      cond_.notify_one();
    }
  }

  // Operations related to the sync results
  bool empty() {
    std::lock_guard<std::mutex> lock(m_);
    return sync_queue_.empty();
  }

  size_t size() {
    std::lock_guard<std::mutex> lock(m_);
    return sync_queue_.size();
  }

  void clear() {
    std::lock_guard<std::mutex> lock(m_);
    for (int i = 0; i < num_; ++i) {
      queues_[i].clear();
    }
    sync_queue_.clear();
  }

  void pop_front() {
    std::lock_guard<std::mutex> lock(m_);
    if (!sync_queue_.empty()) {
      sync_queue_.pop_front();
    }
  }

  SyncT front() {
    std::lock_guard<std::mutex> lock(m_);
    return sync_queue_.front();
  }

  bool wait_and_pop_front(SyncT* sync_elem) {
    std::unique_lock<std::mutex> lock(m_);
    cond_.wait(lock, [this](){ return !sync_queue_.empty() || kill_; });
    if (kill_) {
      return false;
    } else {
      *sync_elem = std::move(sync_queue_.front());
      sync_queue_.pop_front();
      return true;
    }
  }

 private:
  // This function is NOT thread safe
  bool find_one_sync_elem(SyncT* sync_elem) {
    bool src_queue_popped, found;
    do {
      found = sync_src_queue_and_target_queue(&queues_[0],
                                              &queues_[1],
                                              &src_queue_popped);
      // For the first pair of queues, we don't care if the src queue is modified.
      // Initialize src_queue_popped to false for the remaining check
      src_queue_popped = false;
      for (int i = 2; i < num_ && found && !src_queue_popped; ++i) {
        found = sync_src_queue_and_target_queue(&queues_[0],
                                                &queues_[i],
                                                &src_queue_popped);
      }
    } while (src_queue_popped);

    if (found) {
      // The front of all queues should contain the synchronous elements now
      sync_elem->clear();
      sync_elem->reserve(num_);
      for (Container& queue : queues_) {
        XP_CHECK(!queue.empty());
        sync_elem->push_back(queue.front());
        queue.pop_front();
      }
    }
    return found;
  }

  // If returned true, both the front of src_queue and tgt_queue contain the sync elements
  // Ow, return false.
  bool sync_src_queue_and_target_queue(Container* src_queue,
                                       Container* tgt_queue,
                                       bool* src_queue_popped) {
    *src_queue_popped = false;
    bool found = false;
    while (!src_queue->empty() && !tgt_queue->empty()) {
      const T& src_T = src_queue->front();
      const T& tgt_T = tgt_queue->front();
      if (src_T.is_older_than(tgt_T)) {
        src_queue->pop_front();
        *src_queue_popped = true;  // The src_queue is modified.
      } else if (src_T.is_newer_than(tgt_T)) {
        tgt_queue->pop_front();
      } else {
        // Found synced src_T & tgt_T
        found = true;
        break;
      }
    }
    return found;
  }

  // Member variables
  std::vector<Container> queues_;
  std::deque<SyncT> sync_queue_;
  std::mutex m_;
  std::condition_variable cond_;
  int num_;  // The number of shared queues to be synced
  int capacity_;  // The max number of sync results this SyncSharedQueue can hold
  bool kill_;
};
}  // namespace XPDRIVER
#endif  // INCLUDE_DRIVER_HELPER_SYNC_SHARED_QUEUE_H_
