#include "teleop_manipulators/backends/kuka_udp_robot.hpp"

namespace teleop {

KukaUdpRobot::KukaUdpRobot(const RobotConfig& config) : config_(config) {}

KukaUdpRobot::~KukaUdpRobot() { disconnect(); }

bool KukaUdpRobot::connect() {
  // TODO: open the UDP socket to config_.ip:config_.port with your existing client.
  return false;
}

void KukaUdpRobot::disconnect() noexcept {
  // TODO: close socket. Must not throw.
}

std::optional<JointState> KukaUdpRobot::read() {
  // TODO: return the latest joint state received from the controller.
  return std::nullopt;
}

bool KukaUdpRobot::write(const JointCommand& cmd) {
  // TODO: convert cmd to the KUKA packet format and send it.
  // If the protocol only accepts positions, set mode: position in kuka.yaml.
  (void)cmd;
  return false;
}

void KukaUdpRobot::stop() noexcept {
  // TODO: send "hold current position" (or zero velocity).
}

}  // namespace teleop
