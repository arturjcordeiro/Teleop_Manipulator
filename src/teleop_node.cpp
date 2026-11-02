#include "teleop_manipulators/teleop_node.hpp"

namespace teleop {

namespace {

Eigen::VectorXd toEigen(const std::vector<double> &v) {
  return Eigen::Map<const Eigen::VectorXd>(v.data(),
                                           static_cast<Eigen::Index>(v.size()));
}

} // namespace

TeleopManipulator::TeleopManipulator(const rclcpp::NodeOptions &options)
    : rclcpp::Node("teleop_manipulator_node", options) {
  ik_params_.tcp_frame = declare_parameter<std::string>("tcp_frame", "tool0");
  ik_params_.base_frame = declare_parameter<std::string>("base_frame", "");
  ik_params_.damping = declare_parameter<double>("ik.damping", 0.05);
  ik_params_.gain_linear = declare_parameter<double>("ik.gain_linear", 2.0);
  ik_params_.gain_angular = declare_parameter<double>("ik.gain_angular", 2.0);

  // ---- Pose mapping (does not depend on the robot model) ----
  PoseMapper::Params map_p;
  const auto R = declare_parameter<std::vector<double>>(
      "mapping.R_base_optitrack", {1, 0, 0, 0, 1, 0, 0, 0, 1});
  if (R.size() != 9) {
    throw std::invalid_argument(
        "mapping.R_base_optitrack must have 9 values (row-major)");
  }
  map_p.R_base_optitrack =
      Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(R.data());
  map_p.translation_scale =
      declare_parameter<double>("mapping.translation_scale", 1.0);
  mapper_ = std::make_unique<PoseMapper>(map_p);

  // ---- Safety parameters (joint limits are filled in from the URDF later)
  // ----
  safety_params_.max_joint_velocity = toEigen(
      declare_parameter<std::vector<double>>("safety.max_joint_velocity"));
  safety_params_.limit_margin =
      declare_parameter<double>("safety.limit_margin", 0.05);
  safety_params_.pose_timeout = std::chrono::milliseconds(
      declare_parameter<int>("safety.pose_timeout_ms", 50));

  // ---- Robot backend parameters ----
  robot_config_.type = declare_parameter<std::string>("robot.type", "mock");
  robot_config_.ip = declare_parameter<std::string>("robot.ip", "");
  robot_config_.port = declare_parameter<int>("robot.port", 0);
  robot_config_.period = declare_parameter<double>("robot.period", 0.002);
  robot_config_.mode =
      declare_parameter<std::string>("robot.mode", "position") == "velocity"
          ? CommandMode::Velocity
          : CommandMode::Position;
  robot_config_.servo_lookahead =
      declare_parameter<double>("robot.servo_lookahead", 0.1);
  robot_config_.servo_gain =
      declare_parameter<double>("robot.servo_gain", 300.0);
  robot_config_.acceleration =
      declare_parameter<double>("robot.acceleration", 1.0);
  robot_config_.max_joint_step =
      declare_parameter<double>("robot.max_joint_step", 0.1);
  robot_config_.kuka_interface =
      declare_parameter<std::string>("robot.kuka_interface", "udp");
  robot_config_.kuka_fri_ip =
      declare_parameter<std::string>("robot.fri_ip", "192.170.10.2");
  robot_config_.kuka_fri_port =
      declare_parameter<int>("robot.fri_port", 30200);
  robot_config_.kuka_command =
      declare_parameter<std::string>("robot.kuka_command", "joint");
  robot_config_.kuka_speed_rel =
      declare_parameter<double>("robot.kuka_speed_rel", 1.0);
  robot_config_.kuka_accel_rel =
      declare_parameter<double>("robot.kuka_accel_rel", 1.0);
  robot_config_.kuka_speed_mm_s =
      declare_parameter<double>("robot.kuka_speed_mm_s", 200.0);
  robot_config_.kuka_accel_mm_s2 =
      declare_parameter<double>("robot.kuka_accel_mm_s2", 0.0);

  pose_topic_ =
      declare_parameter<std::string>("pose_topic", "/optitrack/tool/pose");

  RCLCPP_INFO(get_logger(), "Pose topic: %s ", pose_topic_.c_str());

  // ---- Wait for the robot model ----
  // robot_state_publisher publishes robot_description as transient_local
  // ("latched"), so we receive it even if it was published before we started.
  const auto description_topic = declare_parameter<std::string>(
      "robot_description_topic", "/robot_description");
  description_sub_ = create_subscription<std_msgs::msg::String>(
      description_topic, rclcpp::QoS(1).transient_local().reliable(),
      [this](const std_msgs::msg::String::ConstSharedPtr &msg) {
        onRobotDescription(msg);
      });

  RCLCPP_INFO(get_logger(), "Waiting for robot description on %s ...",
              description_topic.c_str());
}

TeleopManipulator::~TeleopManipulator() {
  running_ = false;
  if (control_thread_.joinable()) {
    control_thread_.join();
  }
  if (robot_) {
    robot_->stop();
    robot_->disconnect();
  }
}

void TeleopManipulator::onRobotDescription(
    const std_msgs::msg::String::ConstSharedPtr &msg) {
  if (robot_) {
    return; // already initialized; ignore re-publications
  }
  RCLCPP_INFO(get_logger(), "Found robot description model");

  // ---- IK from the published URDF ----
  ik_params_.urdf_xml = msg->data;
  ik_ = std::make_unique<DiffIK>(ik_params_);

  // ---- Safety (joint limits from the URDF) ----
  safety_params_.lower_limits = ik_->lowerLimits();
  safety_params_.upper_limits = ik_->upperLimits();
  // The configured velocity vector only applies if it matches the manipulator.
  // Otherwise fall back to the URDF limits so a new arm needs no config edit.
  if (safety_params_.max_joint_velocity.size() !=
      static_cast<Eigen::Index>(ik_->dof())) {
    RCLCPP_WARN(
        get_logger(),
        "safety.max_joint_velocity has %zu values but the manipulator "
        "has %zu joints; using the URDF velocity limits",
        static_cast<std::size_t>(safety_params_.max_joint_velocity.size()),
        ik_->dof());
    safety_params_.max_joint_velocity = ik_->velocityLimits();
  }
  safety_ = std::make_unique<Safety>(safety_params_);

  // ---- Robot backend ----
  robot_config_.dof = ik_->dof();
  // KUKA cartesian mode reuses the IK model instead of building a second one.
  robot_config_.forward_kinematics = [this](const Eigen::VectorXd &q) {
    return ik_->forwardKinematics(q);
  };
  robot_ = makeRobot(robot_config_);
  if (!robot_->connect()) {
    const std::string &ip =
        robot_config_.type == "kuka" && robot_config_.kuka_interface == "fri"
            ? robot_config_.kuka_fri_ip
            : robot_config_.ip;
    throw std::runtime_error("Could not connect to robot (" +
                             robot_config_.type + " @ " + ip + ")");
  }

  // ---- Start teleop ----
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr &m) {
        poseCallback(m);
      });

  clutch_srv_ = create_service<std_srvs::srv::SetBool>(
      "~/clutch",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
             std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        clutchCallback(req, res);
      });

  control_thread_ = std::thread([this] { controlLoop(); });
  description_sub_.reset(); // no longer needed

  RCLCPP_INFO(get_logger(),
              "Teleop ready (%s, %zu DOF). Call ~/clutch to engage.",
              robot_config_.type.c_str(), robot_config_.dof);
}

void TeleopManipulator::poseCallback(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg) {
  const auto &p = msg->pose;
  StampedPose sp;
  sp.pose.translation() =
      Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  sp.pose.linear() = Eigen::Quaterniond(p.orientation.w, p.orientation.x,
                                        p.orientation.y, p.orientation.z)
                         .normalized()
                         .toRotationMatrix();
  // Local receive time: OptiTrack's clock may differ from ours.
  sp.stamp = Clock::now();

  std::lock_guard<std::mutex> lock(pose_mutex_);
  latest_pose_ = sp;
}

void TeleopManipulator::clutchCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
  clutch_request_ = req->data;
  res->success = true;
  res->message = req->data ? "clutch engage requested" : "clutch released";
}

std::optional<StampedPose> TeleopManipulator::latestPose() const {
  std::lock_guard<std::mutex> lock(pose_mutex_);
  return latest_pose_;
}

void TeleopManipulator::controlLoop() {
  const auto period = std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(robot_->period()));
  auto next = Clock::now();

  while (running_ && rclcpp::ok()) {
    next += period;
    step();
    std::this_thread::sleep_until(next);
  }
  robot_->stop();
}

void TeleopManipulator::step() {
  const auto state = robot_->read();
  if (!state) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                          "Lost robot connection");
    disengage();
    return;
  }

  // request latest pose from Optitrack/tool topic
  const auto pose = latestPose();
  // Verify if pose is fresh
  const bool pose_ok = pose && safety_->poseIsFresh(pose->stamp, Clock::now());

  // ---- Clutch state machine ----
  if (!clutch_request_ || !pose_ok) {
    if (engaged_ && !pose_ok) {
      RCLCPP_WARN(get_logger(), "OptiTrack pose stale: stopping");
    }
    disengage();
    return;
  }
  if (!engaged_) { // rising edge: anchor tool pose to current TCP pose
    mapper_->engage(pose->pose, ik_->forwardKinematics(state->q));
    q_cmd_ = state->q;
    engaged_ = true;
  }

  // ---- Mapping -> IK -> safety -> robot ----
  const auto target = mapper_->target(pose->pose);
  if (!target) {
    return;
  }

  if (robot_->commandMode() == CommandMode::Position) {
    // Solve from the commanded q (not measured) for a smooth integrated
    // trajectory.
    Eigen::VectorXd dq = ik_->solve(q_cmd_, *target);
    safety_->clampVelocity(dq);
    q_cmd_ += dq * robot_->period();
    safety_->clampPosition(q_cmd_);
    robot_->write({CommandMode::Position, q_cmd_});
  } else {
    Eigen::VectorXd dq = ik_->solve(state->q, *target);
    safety_->clampVelocity(dq);
    robot_->write({CommandMode::Velocity, dq});
  }
}

void TeleopManipulator::disengage() {
  if (engaged_) {
    mapper_->disengage();
    robot_->stop();
    engaged_ = false;
  }
}

} // namespace teleop

RCLCPP_COMPONENTS_REGISTER_NODE(teleop::TeleopManipulator)
