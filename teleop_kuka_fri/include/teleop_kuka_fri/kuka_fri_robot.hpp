#pragma once

#include "teleop_manipulators_core/manipulator_interface.hpp"

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <thread>

namespace KUKA {
namespace FRI {
class UdpConnection;
class ClientApplication;
} // namespace FRI
} // namespace KUKA

namespace teleop {

class FriTeleopClient; // defined in kuka_fri_robot.cpp

/**
 * KUKA iiwa backend over FRI (KONI interface).
 *
 * The FRI client runs in its own thread at the FRI rate (e.g. 5 ms) and must
 * answer every cycle. The teleop loop (e.g. 120 Hz) only hands over the latest
 * joint target; the FRI thread interpolates between targets and applies a
 * per-cycle velocity limit, so the robot always gets a smooth stream.
 *
 * Robot side: Iiwa_ros2 in POSITION or POSITION (IMPEDANCE) mode.
 * Config: robot.fri_ip = KONI IP of the controller (default 192.170.10.2),
 *         robot.fri_port = FRI port (default 30200),
 *         period = teleop period in seconds (interpolation segment length).
 */
class KukaFriRobot final : public ManipulatorInterface {
public:
  KukaFriRobot() = default;
  ~KukaFriRobot() override;

  KukaFriRobot(const KukaFriRobot &) = delete;
  KukaFriRobot &operator=(const KukaFriRobot &) = delete;

  bool init(const RobotContext &ctx) override;
  bool connect() override;
  void disconnect() noexcept override;
  [[nodiscard]] std::optional<JointState> read() override;
  bool write(const JointCommand &cmd) override;
  void stop() noexcept override;

  [[nodiscard]] CommandMode commandMode() const noexcept override {
    return mode_;
  }
  [[nodiscard]] double period() const noexcept override { return period_; }
  [[nodiscard]] std::size_t dof() const noexcept override { return dof_; }
  [[nodiscard]] std::string lastError() const override { return last_error_; }

private:
  static constexpr std::size_t kDof = 7;

  std::string fri_ip_{"192.170.10.2"};
  int fri_port_{30200};
  double period_{0.002};
  CommandMode mode_{CommandMode::Position};
  std::size_t dof_{0};
  double max_joint_step_{0.1};
  std::string last_error_;

  // Destruction order matters: app_ references connection_ and client_.
  std::unique_ptr<KUKA::FRI::UdpConnection> connection_;
  std::unique_ptr<FriTeleopClient> client_;
  std::unique_ptr<KUKA::FRI::ClientApplication> app_;

  std::thread thread_;
  std::atomic<bool> running_{false};

  std::array<double, kDof> q_cmd_{}; // last target handed to the FRI thread
  bool have_cmd_ = false;
};

} // namespace teleop
