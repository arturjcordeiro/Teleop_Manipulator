// Abstract interface every robot backend implements.
// The node loads a concrete backend through pluginlib and only ever talks to
// this interface, never to a concrete driver.
#pragma once

#include <cstddef>
#include <functional>
#include <optional>
#include <thread>

#include "teleop_manipulators_core/types.hpp"

namespace rclcpp {
class Node;
}

namespace teleop {

// Everything a backend needs from the host, supplied once at init() time.
// A driver declares and reads its own `robot.*` parameters through `node`,
// so the node stays free of any driver-specific configuration.
struct RobotContext {
  rclcpp::Node &node;
  std::size_t dof{0};
  // Optional FK for drivers that command in Cartesian space (KUKA SmartServo).
  std::function<Eigen::Isometry3d(const Eigen::VectorXd &)> forward_kinematics;
};

class ManipulatorInterface {
public:
  virtual ~ManipulatorInterface() = default;

  // Non-copyable: a backend owns a live connection.
  ManipulatorInterface(const ManipulatorInterface &) = delete;
  ManipulatorInterface &operator=(const ManipulatorInterface &) = delete;

  // Called once after loading (pluginlib needs a default constructor).
  virtual bool init(const RobotContext &ctx) {
    (void)ctx;
    return true;
  }

  virtual bool connect() = 0;
  virtual void disconnect() noexcept = 0;

  // Latest measured state, or nullopt if the connection is lost.
  [[nodiscard]] virtual std::optional<JointState> read() = 0;

  // Send one command. Must be called once per period().
  virtual bool write(const JointCommand &cmd) = 0;

  // Bring the robot to a controlled stop. Must never throw.
  virtual void stop() noexcept = 0;

  [[nodiscard]] virtual CommandMode commandMode() const noexcept = 0;
  [[nodiscard]] virtual double period() const noexcept = 0; // [s]
  [[nodiscard]] virtual std::size_t dof() const noexcept = 0;
  [[nodiscard]] virtual std::string lastError() const { return {}; }

  using CycleToken = Clock::time_point;

  // Pacing hook. Backends with their own precise cycle timer override these;
  // everything else keeps the default fixed-rate sleep.
  virtual CycleToken beginCycle() { return Clock::now(); }

  virtual void endCycle(CycleToken t0) {
    std::this_thread::sleep_until(t0 +
                                  std::chrono::duration_cast<Clock::duration>(
                                      std::chrono::duration<double>(period())));
  }

protected:
  ManipulatorInterface() = default;
};

} // namespace teleop
