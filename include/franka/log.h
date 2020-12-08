// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <iterator>
#include <sstream>
#include <vector>

#include <franka/control_types.h>
#include <franka/robot_state.h>

/**
 * @file log.h
 * Contains helper types for logging sent commands and received robot states.
 */

namespace franka {

namespace internal {

template <typename T, size_t N>
std::string csvName(const std::array<T, N>& /*unused*/, const std::string& name) {
  std::ostringstream os;
  for (size_t i = 0; i < N - 1; i++) {
    os << name << "[" << i << "], ";
  }
  os << name << "[" << N - 1 << "]";
  return os.str();
}

template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream /*unused*/, const std::array<T, N>& array) {
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  return ostream;
}

}  // namespace internal

/**
 * Command sent to the robot. Structure used only for logging purposes.
 */
struct RobotCommand {
  /**
   * \f$q_d\f$ sent to the robot.
   */
  JointPositions joint_positions{0, 0, 0, 0, 0, 0, 0};
  /**
   * \f$\dot{q}_d\f$ sent to the robot.
   */
  JointVelocities joint_velocities{0, 0, 0, 0, 0, 0, 0};
  /**
   * \f$^O{\mathbf{T}_{EE}}_{d}\f$ sent to the robot.
   */
  CartesianPose cartesian_pose{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  /**
   * \f$^O\dot{P}_{EE}\f$ sent to the robot.
   */
  CartesianVelocities cartesian_velocities{0, 0, 0, 0, 0, 0};
  /**
   * \f${\tau_J}_d\f$ sent to the robot.
   */
  Torques torques{0, 0, 0, 0, 0, 0, 0};
};

/**
 * One row of the log contains a robot command of timestamp n and a
 * corresponding robot state of timestamp n+1.
 * Provided by the ControlException.
 */
struct Record {
  /**
   * Robot state of timestamp n+1.
   */
  RobotState state;
  /**
   * Robot command of timestamp n, after rate limiting (if activated).
   */
  RobotCommand command;
};

/**
 * Writes the log to a string in CSV format. If the string is not empty, the first row contains the
 * header with names of columns. The following lines contain rows of values separated by commas.
 *
 * If the log is empty, the function returns an empty string.
 *
 * @param[in] log Log provided by the ControlException.
 *
 * @return a string in CSV format, or empty string.
 */
std::string logToCSV(const std::vector<Record>& log);
}  // namespace franka
