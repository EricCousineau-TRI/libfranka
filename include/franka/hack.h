#pragma once

#undef NDEBUG

#include <cassert>

#include <chrono>
#include <iomanip>
#include <queue>
#include <sstream>
#include <vector>

#include <franka/log.h>

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

struct HackEntry {
  // Host time.
  double host_time{};

  // From robot state. Resolution is only milliseconds.
  double time{};
  double success_rate{};

  // Measured.
  std::array<double, 7> q;
  std::array<double, 7> dq;
  std::array<double, 7> tau_J;
  std::array<double, 7> tau_ext_hat_filtered;
  // From robot state.
  std::array<double, 7> q_d;
  std::array<double, 7> dq_d;

  // Commanded (but before low-pass filters / rate limiting).
  std::array<double, 7> q_c;

  // Delay value. Only using this to see if processor is pruning execution
  // branches?
  double delta_angle_delayed{};
};

std::string logToCSV(const std::vector<HackEntry>& log) {
  using internal::csvName;
  using internal::operator<<;

  assert(log.size() > 0);
  const auto& first = log.front();
  std::ostringstream os;
  os
    << std::setprecision(std::numeric_limits<double>::digits10 + 1)
    << "host_time, "
    << "time, "
    << "success_rate, "
    << csvName(first.q, "q") << ", "
    << csvName(first.dq, "dq") << ", "
    << csvName(first.tau_J, "tau_J") << ", "
    << csvName(first.tau_ext_hat_filtered, "tau_ext_hat_filtered") << ", "
    << csvName(first.q_d, "q_d") << ", "
    << csvName(first.dq_d, "dq_d") << ", "
    << csvName(first.q_c, "q_c") << ", "
    << "delta_angle_delayed" << std::endl;
  for (const HackEntry& r : log) {
    os
      << r.host_time << ", "
      << r.time << ", "
      << r.success_rate << ", "
      << r.q << ", "
      << r.dq << ", "
      << r.tau_J << ", "
      << r.tau_ext_hat_filtered << ", "
      << r.q_d << ", "
      << r.dq_d << ", "
      << r.q_c << ", "
      << r.delta_angle_delayed << std::endl;
  }
  return os.str();
}

double CurrentTimeSeconds() {
  return std::chrono::duration<double>(
      std::chrono::system_clock::now().time_since_epoch()).count();
}

}  // namespace franka
