// Abstract interface every robot backend implements.
// The control loop only talks to this, never to a concrete robot.
#pragma once

#include <cstddef>
#include <optional>
#include <thread>

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

  using CycleToken = Clock::time_point;

  // Pacing hook. Backends with their own precise cycle timer override these;
  // everything else keeps the default fixed-rate sleep.
  virtual CycleToken beginCycle() { return Clock::now(); }

  virtual void endCycle(CycleToken t0) {
    std::this_thread::sleep_until(
        t0 + std::chrono::duration_cast<Clock::duration>(
                 std::chrono::duration<double>(period())));
  }

protected:
  RobotInterface() = default;
};

} // namespace teleop
