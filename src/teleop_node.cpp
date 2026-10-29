#include "teleop_manipulators/teleop_node.hpp"

namespace teleop {

namespace {
constexpr double kMinControlRateHz = 1.0;
constexpr double kDefaultControlRateHz = 100.0;

/// Clamp a value into [-limit, +limit]. A non-positive limit disables clamping.
double ClampStep(double value, double limit) {
  if (limit <= 0.0) {
    return value;
  }
  return std::clamp(value, -limit, limit);
}
} // namespace
//
Eigen::VectorXd toEigen(const std::vector<double> &v) {
  return Eigen::Map<const Eigen::VectorXd>(v.data(),
                                           static_cast<Eigen::Index>(v.size()));
}

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

  // ---------------------------------------------------------------------------
  // ROS interfaces.
  //
  // The pose subscriber lives in its own mutually exclusive callback group so a
  // slow robot-side callback can never stall pose ingestion.
  // ---------------------------------------------------------------------------
  pose_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,
                            /*automatically_add_to_executor_with_node=*/false);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = pose_callback_group_;

  // Best effort, depth 1: we always want the newest pose, never a backlog.
  tool_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TeleopManipulator::ToolPoseCallback, this,
                std::placeholders::_1),
      sub_options);

  command_topic_ = "iiwa";
  robot_command_publisher_ = create_publisher<ibot_common::msg::ArmCommand>(
      command_topic_, rclcpp::QoS(1));

  // ---------------------------------------------------------------------------
  // Fixed-rate control loop. Publishing from a timer instead of from the pose
  // callback decouples the (jittery) input rate from the (smooth) output rate.
  // ---------------------------------------------------------------------------
  control_rate_hz_ = 60;
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / control_rate_hz_));
  control_timer_ = create_wall_timer(
      period, std::bind(&TeleopManipulator::PublishManipulatorCommand, this));
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
  robot_ = makeRobot(robot_config_);
  if (!robot_->connect()) {
    throw std::runtime_error("Could not connect to robot (" +
                             robot_config_.type + " @ " + robot_config_.ip +
                             ")");
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

void TeleopManipulator::ToolPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  // Hot path: only store the sample. No computation, no logging, no
  // allocation. std::lock_guard<std::mutex> lock(pose_mutex_); latest_pose_ =
  // *msg; latest_pose_stamp_ = msg->header.stamp; has_pose_ = true;
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

void TeleopManipulator::CalculateDelta(geometry_msgs::msg::PoseStamped &pose) {

  // First valid sample: nothing to differentiate against yet.
  if (!has_previous_pose_) {
    previous_pose_ = pose.pose;
    has_previous_pose_ = true;
    return;
  }

  // --- Translation delta -----------------------------------------------------
  double dx = pose.pose.position.x - previous_pose_.position.x;
  double dy = pose.pose.position.y - previous_pose_.position.y;
  double dz = pose.pose.position.z - previous_pose_.position.z;

  // --- Rotation delta --------------------------------------------------------
  // Relative rotation expressed as a rotation vector. Euler angles are never
  // interpolated here.
  const double q1x = previous_pose_.orientation.x;
  const double q1y = previous_pose_.orientation.y;
  const double q1z = previous_pose_.orientation.z;
  const double q1w = previous_pose_.orientation.w;

  const double q2x = pose.pose.orientation.x;
  const double q2y = pose.pose.orientation.y;
  const double q2z = pose.pose.orientation.z;
  const double q2w = pose.pose.orientation.w;

  // dq = q2 * conjugate(q1)
  const double dqw = q2w * q1w + q2x * q1x + q2y * q1y + q2z * q1z;
  const double dqx = -q2w * q1x + q2x * q1w - q2y * q1z + q2z * q1y;
  const double dqy = -q2w * q1y + q2x * q1z + q2y * q1w - q2z * q1x;
  const double dqz = -q2w * q1z - q2x * q1y + q2y * q1x + q2z * q1w;

  // Rotation vector = 2 * atan2(|v|, w) * v / |v|
  double rx = 0.0;
  double ry = 0.0;
  double rz = 0.0;
  const double sin_half = std::sqrt(dqx * dqx + dqy * dqy + dqz * dqz);
  if (sin_half > 1e-9) {
    const double angle = 2.0 * std::atan2(sin_half, std::fabs(dqw));
    const double k = (dqw < 0.0 ? -angle : angle) / sin_half;
    rx = dqx * k;
    ry = dqy * k;
    rz = dqz * k;
  }

  // --- Scaling, deadband and clamping ----------------------------------------
  dx *= translation_scale_;
  dy *= translation_scale_;
  dz *= translation_scale_;

  rx *= rotation_scale_;
  ry *= rotation_scale_;
  rz *= rotation_scale_;

  if (deadband_ > 0.0) {
    if (std::sqrt(dx * dx + dy * dy + dz * dz) < deadband_) {
      dx = 0.0;
      dy = 0.0;
      dz = 0.0;
    }
    if (std::sqrt(rx * rx + ry * ry + rz * rz) < deadband_) {
      rx = 0.0;
      ry = 0.0;
      rz = 0.0;
    }
  }

  dx = ClampStep(dx, max_translation_step_);
  dy = ClampStep(dy, max_translation_step_);
  dz = ClampStep(dz, max_translation_step_);

  rx = ClampStep(rx, max_rotation_step_);
  ry = ClampStep(ry, max_rotation_step_);
  rz = ClampStep(rz, max_rotation_step_);
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

  const double dt = robot_->period();
  if (robot_->commandMode() == CommandMode::Position) {
    // Solve from the commanded q (not measured) for a smooth integrated
    // trajectory.
    Eigen::VectorXd dq = ik_->solve(q_cmd_, *target);
    safety_->clampVelocity(dq);
    q_cmd_ += dq * dt;
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

void TeleopManipulator::PublishManipulatorCommand() {
  //   geometry_msgs::msg::PoseStamped pose;
  //   rclcpp::Time pose_stamp;
  //   bool has_pose = false;
  //
  //   {
  //     std::lock_guard<std::mutex> lock(pose_mutex_);
  //     pose = latest_pose_;
  //     pose_stamp = latest_pose_stamp_;
  //     has_pose = has_pose_;
  //   }
  //
  //   if (!has_pose) {
  //     return;
  //   }
  //
  //   const rclcpp::Time now = this->now();
  //
  //   // Deadman: if the teleoperation device stopped sending poses, command
  //   zero so
  //   // the robot does not keep moving on a stale velocity.
  //   const double pose_age_s =
  //       (pose_stamp.nanoseconds() == 0) ? 0.0 : (now - pose_stamp).seconds();
  //   if (pose_timeout_s_ > 0.0 && pose_age_s > pose_timeout_s_) {
  //     PublishZeroCommand();
  //     has_previous_pose_ = false;
  //     return;
  //   }
  //
  //   // Calculate Delta
  //   TeleopManipulator::CalculateDelta(pose);
  //
  //   // --- Publish
  //   ---------------------------------------------------------------
  //   // The pre-allocated message is reused: no allocation in the control
  //   loop.
  //   // `rclcpp::Time` converts implicitly to `builtin_interfaces::msg::Time`,
  //   so
  //   // the node clock can be assigned straight into the header. Using the
  //   node
  //   // clock (instead of std::chrono / system_clock) keeps the stamp
  //   consistent
  //   // with the rest of the graph and honours `use_sim_time`.
  //   command_msg_.header.stamp = now;
  //   command_msg_.header.frame_id = pose.header.frame_id;
  //
  //   robot_command_publisher_->publish(command_msg_);
  //
  //   previous_pose_ = pose.pose;
  // }
  //
  // void TeleopManipulator::PublishZeroCommand() {
  //   command_msg_.transforms.clear();
  //
  //   // Stamp with the node clock at publish time, exactly like the normal
  //   command
  //   // path, so the driver can order/expire commands consistently.
  //   command_msg_.header.stamp = this->now();
  //   command_msg_.header.frame_id = "";
  //
  //   geometry_msgs::msg::TransformStamped transform;
  //   // Publish all zeros
  //   transform.transform.translation.x = 0.0;
  //   transform.transform.translation.y = 0.0;
  //   transform.transform.translation.z = 0.0;
  //   transform.transform.rotation.x = 0.0;
  //   transform.transform.rotation.y = 0.0;
  //   transform.transform.rotation.z = 0.0;
  //   transform.transform.rotation.w = 1.0;
  //
  //   command_msg_.transforms.push_back(transform);
  //   command_msg_.acceleration = 0.5;
  //   command_msg_.speed = 0.5;
  //
  //   robot_command_publisher_->publish(command_msg_);
}

} // namespace teleop

RCLCPP_COMPONENTS_REGISTER_NODE(teleop::TeleopManipulator)
