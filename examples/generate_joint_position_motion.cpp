// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include "examples_common.h"

/**
 * @example generate_joint_position_motion.cpp
 * An example showing how to generate a joint position motion.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  try {
    franka::Robot robot(argv[1], franka::RealtimeConfig::kIgnore);
    setDefaultBehavior(robot);

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI_4 / 2, 0, -3 * M_PI_4 / 2, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    std::array<double, 7> initial_position;
    double time_high_rate = 0.0;
    double time = 0.0;
    const int frame_delta = 1;
    int frame_count = 0;
    const double T = 5.0;

    auto position_callback =
        [&](const franka::RobotState& robot_state, franka::Duration period)
          -> franka::JointPositions {
      time_high_rate += period.toSec();
      if (frame_count % frame_delta == 0) {
        time = time_high_rate;
        std::cout << "update" << std::endl;
      }
      frame_count += 1;

      std::cout << "period: " << period.toSec() << std::endl;

      if (time == 0.0) {
        initial_position = robot_state.q_d;
      }

      double delta_angle = M_PI / 12.0 * (1 - std::cos(2 * M_PI / T * time));

      franka::JointPositions output = {{initial_position[0], initial_position[1],
                                        initial_position[2], initial_position[3] + delta_angle,
                                        initial_position[4] + delta_angle, initial_position[5],
                                        initial_position[6] + delta_angle}};

      // Hack in rate limit.
      for (int i = 0; i < 7; ++i) {
        const double max_vel = 2;
        const double max_accel = 30;
        const double max_jerk = 10000;
        output.q[i] = franka::limitRate(
            max_vel,
            max_accel,
            max_jerk,
            output.q[i],
            robot_state.q_d[i],
            robot_state.dq_d[i],
            robot_state.ddq_d[i]);
      }

      if (time >= T * 4) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(output);
      }
      return output;
    };
    const franka::ControllerMode controller_mode =
        franka::ControllerMode::kJointImpedance;
    // const bool limit_rate = true;
    // const double cutoff_freq = 1;
    robot.control(position_callback, controller_mode); //, limit_rate, cutoff_freq);
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
