// Plain data types shared by the node and every robot backend. No logic, no ROS.
#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>

namespace teleop {

using Clock = std::chrono::steady_clock;

// Measured robot joint state.
struct JointState {
  Eigen::VectorXd q;   // [rad]
  Eigen::VectorXd dq;  // [rad/s]
  Clock::time_point stamp{};
};

// How a backend wants to be commanded.
enum class CommandMode { Position, Velocity };

// Command sent to a robot backend.
struct JointCommand {
  CommandMode mode{CommandMode::Position};
  Eigen::VectorXd value;  // q [rad] or dq [rad/s], depending on mode
};

// A pose plus the (local, steady-clock) time it was received.
struct StampedPose {
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  Clock::time_point stamp{};
};

}  // namespace teleop
