#undef NDEBUG

// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>
#include <fstream>
#include <limits>

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

const double kNaN = std::numeric_limits<double>::quiet_NaN();
using std::isnan;
const double kEps = 1e-6;

/*
Inputs (slow_freq):
  u[t_slow]

State (slow_freq):
  x[t_slow_prev], x[t_slow]

Output (high_freq):
  if any uninitialized x:
    y[t_fast] = x[t_slow]
  else:
    y[t_fast] = x[t_slow_prev] + (t_fast - t_slow) / dt_slow * (x[t_slow] - x[t_slow_prev])

dt_slow - Slow frequency (seconds)
t_slow[n] - Time at slow frequency
t_fast[n] - Time at fast frequency
*/
class FirstOrderHold {
 public:
  FirstOrderHold(double dt_slow)
      // : dt_slow_(dt_slow)
  {
    (void)dt_slow;
    // t_slow_next_ = 0.0;
  }

  void Update(double t_fast, double u) {
    (void)t_fast;
    // if (t_fast + kEps >= t_slow_next_) {
      // Get first order.
      // t_slow_prev_ = t_slow_;
      x_t_slow_prev_ = x_t_slow_;
      // Get zero-th order.
      // t_slow_ = t_fast;
      x_t_slow_ = u;
      // Schedule next update.
      // t_slow_next_ += dt_slow_;
      count_ += 1;
    // } else {
    //   // hack!
    //   assert(false);
    // }
  }

  double CalcOutput(double t_fast) const {
    (void)t_fast;
    // // Update must be called before output. (Deviates from Drake in
    // // this respect).
    // assert(!isnan(t_slow_) && !isnan(x_t_slow_));
    if (count_ == 1) {
    //   assert(isnan(t_slow_prev_) && isnan(x_t_slow_prev_));
      return x_t_slow_;
    } else if (count_ > 1) {
    //   assert(!isnan(t_slow_prev_) && !isnan(x_t_slow_prev_));
    //   (void)t_fast;
    //   // assert(t_fast >= t_slow_);
    //   // double blend = (t_fast - t_slow_) / dt_slow_;
    //   // const double y = x_t_slow_prev_ + blend * (x_t_slow_ - x_t_slow_prev_);
    //   // assert(blend == 0.0);
    //   // assert(y == x_t_slow_prev_);
    //   // return y;
      // return x_t_slow_prev_;
      return x_t_slow_;
    } else {
      assert(false);
    }
  }

 private:
  int count_{};
  // double dt_slow_{};
  // double t_slow_next_{kNaN};
  // double t_slow_{kNaN};
  double x_t_slow_{0.0};
  // double t_slow_prev_{kNaN};
  double x_t_slow_prev_{0.0};
};

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
    // std::cout << "WARNING: This example will move the robot! "
    //           << "Please make sure to have the user stop button at hand!" << std::endl
    //           << "Press Enter to continue..." << std::endl;
    // std::cin.ignore();
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
    double time_fast = 0.0;
    // double time_slow = 0.0;
    // const int frame_delta = 1;
    // int frame_count = 0;
    const double T = 5.0;

    const double freq_slow = 1000.0; // / frame_delta;
    const double dt_slow = 1.0 / freq_slow;

    FirstOrderHold foh_3(dt_slow);

    auto position_callback =
        [&](const franka::RobotState& robot_state, franka::Duration period)
          -> franka::JointPositions {
      time_fast += period.toSec();
      const double time_slow = time_fast;
      // if (frame_count % frame_delta == 0) {
      //   time_slow = time_fast;
      // }
      // frame_count += 1;

      if (time_fast == 0.0) {
        initial_position = robot_state.q_d;
      }

      double delta_angle = M_PI / 12.0 / 5 * (1 - std::cos(2 * M_PI / T * time_slow));

      franka::JointPositions output = {{
        initial_position[0],
        initial_position[1],
        initial_position[2],
        initial_position[3] + delta_angle,
        initial_position[4],
        initial_position[5],
        initial_position[6]}};

      {
        foh_3.Update(time_fast, output.q[3]);
        const double q3 = foh_3.CalcOutput(time_fast);
        output.q[3] = q3;
      }

      if (time_slow >= T) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(output);
      }
      return output;
    };
    const franka::ControllerMode controller_mode =
        franka::ControllerMode::kJointImpedance;
    robot.control(position_callback, controller_mode);

    // I can haz log.
    const std::string log_file = "/tmp/panda.log";
    {
      std::ofstream log_stream(log_file.c_str());
      log_stream << franka::logToCSV(robot.flushLog());
    }
    std::cout << "Wrote log: " << log_file << "\n";

  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
