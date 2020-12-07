#undef NDEBUG
// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>
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
  u[ts]

State (slow_freq):
  x[ts_prev], x[ts]

Output (high_freq):
  if any uninitialized x:
    y[tf] = x[ts]
  else:
    y[tf] = x[ts_prev] + (tf - ts) / dt_ts * (x[ts] - x[ts_prev])

dt_ts - Slow frequency (seconds)
dt_f - Fast frequency (seconds)
ts[n] - Time at slow frequency
tf[n] - Time at fast frequency
*/
class FirstOrderHold {
 public:
  FirstOrderHold(double dt_ts)
      : dt_ts_(dt_ts) {
    ts_next_ = 0.0;
  }

  void Update(double tf, double u, bool print = false) {
    if (print) std::cout << "update: " << tf << std::endl;
    if (tf + kEps >= ts_next_) {
      if (print) std::cout << "  store" << std::endl;
      // Get first order.
      ts_prev_ = ts_;
      x_ts_prev_ = x_ts_;
      // Get zero-th order.
      ts_ = tf;
      x_ts_ = u;
      // Schedule next update.
      ts_next_ += dt_ts_;
    } else {
      assert(false);
    }
  }

  double CalcOutput(double tf, bool print = false) const {
    // Update must be called before output. (Deviates from Drake in
    // this respect).
    assert(!isnan(ts_));
    assert(!isnan(x_ts_));
    if (isnan(ts_prev_)) {
      assert(isnan(x_ts_prev_));
      return x_ts_;
    } else {
      assert(!isnan(x_ts_prev_));
      assert(tf >= ts_);
      double blend = (tf - ts_) / dt_ts_;
      if (print) std::cout << "blend: " << blend << "\n";
      const double y = x_ts_prev_ + blend * (x_ts_ - x_ts_prev_);
      assert(blend == 0.0);
      assert(y == x_ts_prev_);
      return y;
    }
  }

 private:
  double dt_ts_{};
  double ts_next_{kNaN};
  double ts_{kNaN};
  double x_ts_{kNaN};
  double ts_prev_{kNaN};
  double x_ts_prev_{kNaN};
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
    double time_fast = 0.0;
    double time_slow = 0.0;
    const int frame_delta = 1;
    int frame_count = 0;
    const double T = 5.0;

    const double freq_slow = 1000.0 / frame_delta;
    const double dt_ts = 1.0 / freq_slow;

    std::vector<FirstOrderHold> foh;
    for (int i = 0; i < 7; ++i) {
      foh.emplace_back(dt_ts);
    }

    auto position_callback =
        [&](const franka::RobotState& robot_state, franka::Duration period)
          -> franka::JointPositions {
      time_fast += period.toSec();
      if (frame_count % frame_delta == 0) {
        time_slow = time_fast;
      }
      frame_count += 1;

      if (time_fast == 0.0) {
        initial_position = robot_state.q_d;
      }

      double delta_angle = M_PI / 12.0 * (1 - std::cos(2 * M_PI / T * time_slow));

      franka::JointPositions output = {{initial_position[0], initial_position[1],
                                        initial_position[2],
                                        initial_position[3] + delta_angle,
                                        initial_position[4],// + delta_angle,
                                        initial_position[5],
                                        initial_position[6]}};// + delta_angle}};

      // // Hack in rate limit.
      // for (int i = 0; i < 7; ++i) {
      //   const double max_vel = 2;
      //   const double max_accel = 30;
      //   const double max_jerk = 10000;
      //   output.q[i] = franka::limitRate(
      //       max_vel,
      //       max_accel,
      //       max_jerk,
      //       output.q[i],
      //       robot_state.q_d[i],
      //       robot_state.dq_d[i],
      //       robot_state.ddq_d[i]);
      // }

      for (int i = 0; i < 7; ++i) {
        const bool print = (i == 3);
        foh[i].Update(time_fast, output.q[i], print);
        const double tmp = foh[i].CalcOutput(time_fast, print);
        if (print) {
          std::cout << "q[" << i << "]\n"
              << "  actual: " << output.q[i] << "\n"
              << "  foh: " << tmp << "\n";
        }
        output.q[i] = tmp;
      }

      if (time_slow >= T) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(output);
      }
      return output;
    };
    const franka::ControllerMode controller_mode =
        franka::ControllerMode::kJointImpedance;
    // const bool limit_rate = true;
    // const double cutoftf_freq = 1;
    robot.control(position_callback, controller_mode); //, limit_rate, cutoftf_freq);
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
