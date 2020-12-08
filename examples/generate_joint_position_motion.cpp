// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>

#include <fstream>
#include <iostream>
#include <thread>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/hack.h>

#include "examples_common.h"

using franka::FromString;
using franka::BufferDelay;

/**
 * @example generate_joint_position_motion.cpp
 * An example showing how to generate a joint position motion.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname> <via-time> <num-delay>" << std::endl;
    return -1;
  }
  try {
    franka::Robot robot(argv[1], franka::RealtimeConfig::kIgnore);
    setDefaultBehavior(robot);

    // Inject delay via time (shift reference signal). If false, use a buffer.
    const bool via_time = FromString<bool>(argv[2]);
    // Number of ticks to delay.
    const int num_delay = FromString<int>(argv[3]);

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI_4 / 2, 0, -3 * M_PI_4 / 2, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;
    std::cout << "Waiting to settle (0.5s)..." << std::endl;
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(500));

    std::array<double, 7> initial_position;
    double time = 0.0;
    const double T = 5.0;
    const double dt = 1e-3;

    BufferDelay delay_q3(num_delay);
    std::vector<franka::HackEntry> log;

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

      const double q3_discrete_delay = delay_q3.Step(output.q[3]);
      if (!via_time) {
        output.q[3] = q3_discrete_delay;
      }

      // Log. (Not realtime, but meh.)
      franka::HackEntry entry;
      entry.host_time = franka::CurrentTimeSeconds();
      entry.time = time;
      entry.success_rate = robot_state.control_command_success_rate;
      entry.q = robot_state.q;
      entry.dq = robot_state.dq;
      entry.tau_J = robot_state.tau_J;
      entry.tau_ext_hat_filtered = robot_state.tau_ext_hat_filtered;
      entry.q_d = robot_state.q_d;
      entry.dq_d = robot_state.dq_d;
      entry.q_c = output.q;
      log.push_back(entry);

      if (time >= T) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(output);
      }
      return output;
    };

    std::cout << "Running" << std::endl;
    try {
      robot.control(position_callback);
    } catch (const franka::ControlException& e) {
      std::cout << "ControlException: " << e.what() << std::endl;
    }

    // Save log file.
    // Expects to be run from `<repo>/build`.
    const std::string log_file =
        "../tmp/data/delay_" + std::to_string(num_delay) + ".log";
    std::ofstream log_stream(log_file.c_str());
    log_stream << franka::logToCSV(log);
    std::cout << "Wrote log: " << log_file << "\n";
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
