#pragma once

#undef NDEBUG

#include <cassert>

#include <queue>
#include <sstream>

namespace franka {

// Injects a certain amount of delay.
class BufferDelay {
 public:
  BufferDelay(int num_delay) : max_size_(num_delay + 1) {
    assert(num_delay >= 0);
  }

  double Step(double u) {
    // Update.
    if (buffer_.size() == max_size_) {
      buffer_.pop();
    }
    buffer_.push(u);
    assert(buffer_.size() <= max_size_);
    // Output.
    return buffer_.front();
  }

 private:
  size_t max_size_;
  std::queue<double> buffer_;
};

template <typename T>
T FromString(const std::string& s) {
  std::istringstream ss(s);
  T value{};
  ss >> value;
  assert(!ss.fail());
  return value;
}

}  // namespace franka
