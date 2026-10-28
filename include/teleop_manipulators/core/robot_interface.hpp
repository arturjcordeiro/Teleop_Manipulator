// Abstract interface every robot backend implements.
// The control loop only talks to this, never to a concrete robot.
#pragma once

#include <cstddef>
#include <optional>

#include "teleop_manipulators/core/types.hpp"

namespace teleop {

class RobotInterface {
public:
  virtual ~RobotInterface() = default;

  // Non-copyable: a backend owns a live connection.
  RobotInterface(const RobotInterface &) = delete;
  RobotInterface &operator=(const RobotInterface &) = delete;

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

protected:
  RobotInterface() = default;
};

} // namespace teleop
