/******************************************************************************
 * Copyright 2017-2018 Baidu Robotic Vision Authors. All Rights Reserved.
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
#ifndef INCLUDE_DRIVER_HELPER_RING_BUFFER_H_
#define INCLUDE_DRIVER_HELPER_RING_BUFFER_H_

/*
 * Implementation of a simple ring_buffer class
 */

#include <memory>

namespace XPDRIVER {

template <typename T>
class RingBuffer {
 public:
  RingBuffer() :
  buf_(nullptr),
      head_(0), last_(0), capacity_(0), size_(0) {}
  explicit RingBuffer(const size_t capacity) :
      buf_(std::unique_ptr<T[]>(new T[capacity])),
      head_(0), last_(0), capacity_(capacity), size_(0) {}
  T front() const {
    // It's the caller's responsibility to check for empty buffer
    return buf_[head_];
  }
  T& front() {
    // It's the caller's responsibility to check for empty buffer
    return buf_[head_];
  }
  T back() const {
    // It's the caller's responsibility to check for empty buffer
    return buf_[last_];
  }
  T& back() {
    // It's the caller's responsibility to check for empty buffer
    return buf_[last_];
  }
  T second_to_last() const {
    // It's the caller's responsibility to check for empty buffer
    size_t index = (last_ + capacity_ - 1) % capacity_;
    return buf_[index];
  }
  T& second_to_last() {
    // It's the caller's responsibility to check for empty buffer
    size_t index = (last_ + capacity_ - 1) % capacity_;
    return buf_[index];
  }
  bool set_capacity(size_t capacity) {
    buf_.reset(new T[capacity]);
    capacity_ = capacity;
    return buf_ != nullptr;
  }
  size_t size() const {
    return size_;
  }
  bool empty() const {
    return size_ == 0;
  }
  bool full() const {
    return size_ == capacity_;
  }
  void push_back(T value) {
    if (full()) {
      last_ = head_;
      buf_[last_] = value;
      head_ = (head_ + 1) % capacity_;
    } else if (empty()) {
      last_ = head_;
      buf_[last_] = value;
      size_++;
    } else {
      last_ = (last_ + 1) % capacity_;
      buf_[last_] = value;
      size_++;
    }
  }
  T  pop_front() {
    // It's the caller's responsibility to check for empty buffer
    T ret = buf_[head_];
    size_--;
    head_ = (head_ + 1) % capacity_;
    return ret;
  }
  T& operator[] (size_t index) {
    // It's the caller's responsibility to check for empty buffer
    return buf_[(index + head_) % capacity_];
  }
  const T& operator[] (size_t index) const {
    // It's the caller's responsibility to check for empty buffer
    return buf_[(index + head_) % capacity_];
  }

 private:
  std::unique_ptr<T[]> buf_;
  size_t head_;  // THe location of the first element
  size_t last_;  // The location of the last element
  size_t capacity_;
  size_t size_;
};

}  // namespace XPDRIVER
#endif  // INCLUDE_DRIVER_HELPER_RING_BUFFER_H_
