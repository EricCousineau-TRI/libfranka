#undef NDEBUG

// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>
#include <fstream>
#include <queue>
#include <sstream>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"

/**
 * @example generate_joint_position_motion.cpp
 * An example showing how to generate a joint position motion.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

// Injects a certain amount of delay.
class Delay {
 public:
  Delay(int num_delay) : max_size_(num_delay + 1) {
    assert(num_delay >= 0);
  }

  void Update(double u) {
    if (buffer_.size() == max_size_) {
      buffer_.pop();
    }
    buffer_.push(u);
    assert(buffer_.size() <= max_size_);
  }

  double CalcOutput() const {
    return buffer_.front();
  }

 private:
  size_t max_size_;
  std::queue<double> buffer_;
};

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname> <num-delay>" << std::endl;
    return -1;
  }
  try {
    franka::Robot robot(argv[1], franka::RealtimeConfig::kIgnore);
    setDefaultBehavior(robot);

    int num_delay{};
    {
      std::istringstream ss(argv[2]);
      ss >> num_delay;
      assert(!ss.fail());
    }
    std::cout << "num_delay: " << num_delay << "\n";

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI_4 / 2, 0, -3 * M_PI_4 / 2, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    robot.control(motion_generator);
    robot.flushLog();
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    std::array<double, 7> initial_position;
    double time = 0.0;
    const double T = 5.0;

    Delay delay_3(num_delay);

    auto position_callback =
        [&](const franka::RobotState& robot_state, franka::Duration period)
          -> franka::JointPositions {
      time += period.toSec();
      if (time == 0.0) {
        initial_position = robot_state.q_d;
      }

      double delta_angle = M_PI / (12.0 * 5) * (1 - std::cos(2 * M_PI / T * time));

      franka::JointPositions output = {{
        initial_position[0],
        initial_position[1],
        initial_position[2],
        initial_position[3] + delta_angle,
        initial_position[4],
        initial_position[5],
        initial_position[6]}};

      {
        delay_3.Update(output.q[3]);
        const double q3 = delay_3.CalcOutput();
        output.q[3] = q3;
      }

      if (time >= T) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(output);
      }
      return output;
    };

    std::vector<franka::Record> log;
    try {
      robot.control(position_callback);
    } catch (const franka::ControlException& e) {
      std::cout << "ControlException: " << e.what() << std::endl;
      log = e.log;
    }

    if (log.size() == 0) {
      // No error was triggered, but still get the most recent log.
      log = robot.flushLog();
    }

    // Save log file.
    const std::string log_file =
        "/home/eacousineau/data/panda/interp/delay_" + std::to_string(num_delay) + ".log";
    std::ofstream log_stream(log_file.c_str());
    log_stream << franka::logToCSV(log);
    std::cout << "Wrote log: " << log_file << "\n";
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
