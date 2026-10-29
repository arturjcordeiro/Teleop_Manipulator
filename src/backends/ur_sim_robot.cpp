#include "teleop_manipulators/backends/ur_sim_robot.hpp"

#include <algorithm>
#include <chrono>

namespace teleop {

namespace {
// Same order as RTDE getActualQ(): base -> wrist 3.
// Prepend your tf_prefix if you launched with one.
const std::array<std::string, 6> kJointNames = {
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint"};

constexpr char kJointStateTopic[] = "/joint_states";
constexpr char kPositionTopic[] = "/forward_position_controller/commands";
constexpr char kVelocityTopic[] = "/forward_velocity_controller/commands";

constexpr std::chrono::milliseconds kStateTimeout{200};
constexpr std::chrono::seconds kConnectTimeout{3};
} // namespace

UrSimRobot::UrSimRobot(const RobotConfig &config) : config_(config) {}

UrSimRobot::~UrSimRobot() { disconnect(); }

// --------------------------------------------------------------- connection
bool UrSimRobot::connect() {
  if (!rclcpp::ok()) {
    last_error_ = "rclcpp not initialised";
    return false;
  }

  // Own node + executor thread, so this backend works no matter how the
  // control loop is run (same idea as ur_rtde's internal threads).
  node_ = std::make_shared<rclcpp::Node>("teleop_sim_backend");

  sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      kJointStateTopic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        onJointState(msg);
      });

  const char *topic =
      config_.mode == CommandMode::Position ? kPositionTopic : kVelocityTopic;
  cmd_pub_ =
      node_->create_publisher<std_msgs::msg::Float64MultiArray>(topic, 10);

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  spin_thread_ = std::thread([this] { executor_->spin(); });

  // Wait for the first joint state.
  const auto deadline = Clock::now() + kConnectTimeout;
  while (Clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (have_state_)
        break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  if (cmd_pub_->get_subscription_count() == 0) {
    last_error_ = std::string("no controller listening on ") + topic +
                  " (is it active? ros2 control list_controllers)";
    disconnect();
    return false;
  }
  if (!ready()) {
    disconnect();
    return false;
  }
  return true;
}

void UrSimRobot::disconnect() noexcept {
  try {
    stop();
    if (executor_)
      executor_->cancel();
    if (spin_thread_.joinable())
      spin_thread_.join();
  } catch (...) {
  }
  sub_.reset();
  cmd_pub_.reset();
  executor_.reset();
  node_.reset();
  std::lock_guard<std::mutex> lk(mtx_);
  have_state_ = false;
}

bool UrSimRobot::ready() {
  if (!node_) {
    last_error_ = "not connected";
    return false;
  }
  std::lock_guard<std::mutex> lk(mtx_);
  if (!have_state_) {
    last_error_ = "no joint_states received";
    return false;
  }
  if (Clock::now() - last_msg_ > kStateTimeout) {
    last_error_ = "joint_states stale (simulation stopped?)";
    return false;
  }
  return true;
}

// --------------------------------------------------------------- state
void UrSimRobot::onJointState(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  // joint_states order is NOT guaranteed -> map by name.
  Eigen::VectorXd q(6), qd = Eigen::VectorXd::Zero(6);
  for (size_t j = 0; j < kJointNames.size(); ++j) {
    auto it = std::find(msg->name.begin(), msg->name.end(), kJointNames[j]);
    if (it == msg->name.end())
      return; // not our robot's message
    const auto i = static_cast<size_t>(std::distance(msg->name.begin(), it));
    if (i >= msg->position.size())
      return;
    q[j] = msg->position[i];
    if (i < msg->velocity.size())
      qd[j] = msg->velocity[i];
  }
  std::lock_guard<std::mutex> lk(mtx_);
  q_ = q;
  qd_ = qd;
  last_msg_ = Clock::now();
  have_state_ = true;
}

std::optional<JointState> UrSimRobot::read() {
  if (!ready())
    return std::nullopt;
  std::lock_guard<std::mutex> lk(mtx_);
  return JointState{q_, qd_, last_msg_};
}

// --------------------------------------------------------------- command
void UrSimRobot::publish(const Eigen::VectorXd &v) {
  std_msgs::msg::Float64MultiArray msg;
  msg.data.assign(v.data(), v.data() + v.size());
  cmd_pub_->publish(msg);
}

bool UrSimRobot::write(const JointCommand &cmd) {
  if (!ready())
    return false;
  if (cmd.value.size() != 6) {
    last_error_ = "UR expects 6 joint values";
    return false;
  }

  if (cmd.mode == CommandMode::Position) {
    // Same safety check as the real backend, so sim catches the same bugs.
    Eigen::VectorXd q;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      q = q_;
    }
    const double jump = (cmd.value - q).cwiseAbs().maxCoeff();
    if (jump > config_.max_joint_step) {
      last_error_ = "rejected position target: " + std::to_string(jump) +
                    " rad from actual";
      stop();
      return false;
    }
    publish(cmd.value);
    return true;
  }

  // Velocity: emulate speedJ's acceleration limit.
  const double max_dv = config_.acceleration * config_.period;
  const Eigen::VectorXd dv =
      (cmd.value - last_vel_cmd_).cwiseMax(-max_dv).cwiseMin(max_dv);
  last_vel_cmd_ += dv;
  publish(last_vel_cmd_);
  return true;
}

void UrSimRobot::stop() noexcept {
  try {
    if (!cmd_pub_)
      return;
    if (config_.mode == CommandMode::Position) {
      std::lock_guard<std::mutex> lk(mtx_);
      if (have_state_)
        publish(q_); // hold current pose
    } else {
      last_vel_cmd_.setZero();
      publish(last_vel_cmd_);
    }
  } catch (...) {
  }
}

} // namespace teleop
