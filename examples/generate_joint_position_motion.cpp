// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>
#include <fstream>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/hack.h>

#include "examples_common.h"

using franka::FromString;
using franka::Delay;

/**
 * @example generate_joint_position_motion.cpp
 * An example showing how to generate a joint position motion.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

int main(int argc, char** argv) {
  if (argc != 5) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname> <via-time> <keep-latest> <num-delay>" << std::endl;
    return -1;
  }
  try {
    franka::Robot robot(argv[1], franka::RealtimeConfig::kIgnore);
    setDefaultBehavior(robot);

    // Inject delay via time (shift reference signal). If false, use a buffer.
    const bool via_time = FromString<bool>(argv[2]);
    // Regardless of computation, still keep the latest reference signal (only used if via_time=false).
    const bool keep_latest = FromString<bool>(argv[3]);
    // Number of ticks to delay.
    const int num_delay = FromString<int>(argv[4]);

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI_4 / 2, 0, -3 * M_PI_4 / 2, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    robot.control(motion_generator);
    robot.flushLog();
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    std::array<double, 7> initial_position;
    double time = 0.0;
    const double T = 5.0;
    const double dt = 1e-3;

    Delay delay_3(num_delay);

    auto position_callback =
        [&](const franka::RobotState& robot_state, franka::Duration period)
          -> franka::JointPositions {
      time += period.toSec();
      if (time == 0.0) {
        initial_position = robot_state.q_d;
      }

      double ref_time = time;
      if (via_time) {
        // Inject saturated delay (continuous-time transfer function delay).
        ref_time = std::max(0.0, time - dt * num_delay);
      }

      double delta_angle = M_PI / (12.0 * 5) * (1 - std::cos(2 * M_PI / T * ref_time));

      franka::JointPositions output = {{
        initial_position[0],
        initial_position[1],
        initial_position[2],
        initial_position[3] + delta_angle,
        initial_position[4],
        initial_position[5],
        initial_position[6]}};

      if (!via_time) {
        // Store discrete values (discrete-time transfer function delay).
        const double q3 = delay_3.Step(output.q[3]);
        if (!keep_latest) {
          output.q[3] = q3;
        }
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
