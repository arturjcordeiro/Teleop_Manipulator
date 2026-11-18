#include "teleop_manipulators/teleop_node.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <fcntl.h>

namespace teleop {

namespace {

Eigen::VectorXd toEigen(const std::vector<double> &v) {
  return Eigen::Map<const Eigen::VectorXd>(v.data(),
                                           static_cast<Eigen::Index>(v.size()));
}

} // namespace

TeleopManipulator::TeleopManipulator(const rclcpp::NodeOptions &options)
    : rclcpp::Node("teleop_manipulators_node", options),
      robot_loader_("teleop_manipulators_core", "teleop::ManipulatorInterface") {
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

  // ---- Robot backend selection ----
  robot_type_ = declare_parameter<std::string>("robot.type", "mock");
  max_joint_step_ = declare_parameter<double>("robot.max_joint_step", 0.1);
  publish_joint_states_ =
      declare_parameter<bool>("publish_joint_states", false);
  joint_state_rate_ =
      declare_parameter<double>("joint_state_publish_rate", 50.0);
  if (publish_joint_states_) {
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::QoS(10));
  }

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

  if (publish_joint_states_) {
    joint_names_ = ik_->jointNames();
  }

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
  try {
    robot_ = robot_loader_.createSharedInstance(robot_type_);
  } catch (const std::exception &e) {
    throw std::runtime_error("No robot backend plugin '" + robot_type_ +
                             "'. Is its package installed? (" + e.what() + ")");
  }
  // KUKA cartesian mode reuses the IK model instead of building a second one.
  RobotContext ctx{*this, ik_->dof(), [this](const Eigen::VectorXd &q) {
                     return ik_->forwardKinematics(q);
                   }};
  if (!robot_->init(ctx)) {
    throw std::runtime_error("Robot backend '" + robot_type_ +
                             "' failed to initialize");
  }
  if (!robot_->connect()) {
    throw std::runtime_error("Could not connect to robot (" + robot_type_ +
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
              robot_type_.c_str(), robot_->dof());
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
  while (running_ && rclcpp::ok()) {
    const auto t0 = robot_->beginCycle();
    step();
    robot_->endCycle(t0);
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
  publishJointState(*state);

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
    // Integrate from the commanded q for a smooth trajectory, but bound how
    // far the command may lead the measured joints. Unbounded integration
    // winds up while the arm cannot keep up: servoJ then brakes, and a target
    // >max_joint_step away makes the backend reject it and stop. The bound
    // keeps the target reachable so fast motion continues instead of
    // aborting. (Same rule as the KUKA backends.)
    Eigen::VectorXd dq = ik_->solve(q_cmd_, *target);
    safety_->clampVelocity(dq);
    q_cmd_ += dq * robot_->period();
    safety_->clampPosition(q_cmd_);
    const double max_lead = 0.5 * max_joint_step_;
    const Eigen::VectorXd lo = state->q.array() - max_lead;
    const Eigen::VectorXd hi = state->q.array() + max_lead;
    q_cmd_ = q_cmd_.cwiseMax(lo).cwiseMin(hi);
    if (!robot_->write({CommandMode::Position, q_cmd_})) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Robot write failed: %s",
                           robot_->lastError().c_str());
    }
  } else {
    Eigen::VectorXd dq = ik_->solve(state->q, *target);
    safety_->clampVelocity(dq);
    if (!robot_->write({CommandMode::Velocity, dq})) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Robot write failed: %s",
                           robot_->lastError().c_str());
    }
  }
}

void TeleopManipulator::disengage() {
  if (engaged_) {
    mapper_->disengage();
    robot_->stop();
    engaged_ = false;
  }
}

void TeleopManipulator::publishJointState(const JointState &state) {
  if (!joint_state_pub_ ||
      joint_names_.size() != static_cast<std::size_t>(state.q.size())) {
    return;
  }

  const auto now = Clock::now();
  const auto min_period =
      joint_state_rate_ > 0.0
          ? std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(1.0 / joint_state_rate_))
          : Clock::duration::zero();

  if (now - last_joint_pub_ < min_period) {
    // Did not pass period of refresh rate
    return;
  }
  last_joint_pub_ = now;

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = this->now();
  msg.name = joint_names_;
  msg.position.assign(state.q.data(), state.q.data() + state.q.size());
  msg.velocity.assign(state.dq.data(), state.dq.data() + state.dq.size());
  joint_state_pub_->publish(msg);
}

} // namespace teleop

RCLCPP_COMPONENTS_REGISTER_NODE(teleop::TeleopManipulator)
