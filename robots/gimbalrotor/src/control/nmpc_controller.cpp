#include <gimbalrotor/control/nmpc_controller.h>
#include <gimbalrotor/model/gimbalrotor_robot_model.h>
#include <aerial_robot_estimation/sensor/base_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Cholesky>
#include <Eigen/QR>
#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace gimbalrotor_nmpc
{
namespace
{
using namespace aerial_robot_navigation;
Eigen::Vector3d vector(const tf::Vector3& v) { return {v.x(), v.y(), v.z()}; }
void require(bool valid, const std::string& reason)
{
  if (!valid) throw std::runtime_error("NMPC blocked: " + reason);
}
std::vector<double> values(ros::NodeHandle& nh, const std::string& key, int size)
{
  std::vector<double> v;
  require(nh.getParam(key, v) && v.size() == size, "missing/incorrect array " + key);
  require(std::all_of(v.begin(), v.end(), [](double x) { return std::isfinite(x); }), "nonfinite " + key);
  return v;
}
bool fresh(const ros::Time& stamp, const ros::WallTime& receipt, double timeout)
{
  const double age = (ros::Time::now()-stamp).toSec();
  const double wall_age = (ros::WallTime::now()-receipt).toSec();
  return !stamp.isZero() && !receipt.isZero() && age >= 0 && age < timeout && wall_age >= 0 && wall_age < timeout;
}
Eigen::Quaterniond orientation(const geometry_msgs::Quaternion& q)
{
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
}
}

void Navigator::update()
{
  if (!ready_ && getNaviState() != ARM_OFF_STATE && getNaviState() != STOP_STATE)
    setNaviState(STOP_STATE);
  BaseNavigator::update();
}

void Navigator::naviCallback(const aerial_robot_msgs::FlightNavConstPtr& msg)
{
  const bool tracking = getNaviState() == HOVER_STATE;
  BaseNavigator::naviCallback(msg);
  if (tracking)
  {
    if (msg->roll_nav_mode == aerial_robot_msgs::FlightNav::POS_MODE) setTargetRoll(msg->target_roll);
    if (msg->pitch_nav_mode == aerial_robot_msgs::FlightNav::POS_MODE) setTargetPitch(msg->target_pitch);
  }
}

void Controller::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                            boost::shared_ptr<aerial_robot_model::RobotModel> model,
                            boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                            boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                            double loop_duration)
{
  ControlBase::initialize(nh, nhp, model, estimator, navigator, loop_duration);
  navigation_ = boost::dynamic_pointer_cast<Navigator>(navigator);
  require(bool(navigation_), "the NMPC arming-gated navigator must be selected");
  navigation_->setReady(false);
  ros::NodeHandle n(nh_, "controller/nmpc");
  for (const auto* key : {"enabled", "calibration_verified", "firmware_verified", "emergency_motor_stop_verified"})
  {
    bool confirmed = false;
    require(n.getParam(key, confirmed) && confirmed, std::string(key) + " is false / UNVERIFIED");
  }
  bool real = false, sim = true;
  std::string airframe;
  require(n.getParam("real_machine", real) && real && n.getParam("simulation", sim) && !sim,
          "only the real STM32H7_v2 path is supported");
  require(n.getParam("airframe", airframe) && airframe == "quad", "only quad is supported");
  require(motor_num_ == 4 && model->getJointNum() == 4 &&
          bool(boost::dynamic_pointer_cast<GimbalrotorRobotModel>(model)), "expected four independent tilt joints");
  require(estimate_mode_ == aerial_robot_estimation::EXPERIMENT_ESTIMATE,
          "MVP freshness contract requires the existing IMU + mocap experiment estimator");
  double dt = 0;
  int steps = 0;
  require(n.getParam("sample_time", dt) && n.getParam("steps", steps) && steps == N &&
          std::abs(dt-GIMBALROTOR_NMPC_DT) < 1e-9 && std::abs(loop_duration-dt) < 1e-9,
          "generator, runtime and main_rate disagree");
  const auto qw = values(n, "state_weights", 16), rw = values(n, "input_weights", 8);
  double terminal_scale = 0;
  require(n.getParam("terminal_scale", terminal_scale) && terminal_scale == GIMBALROTOR_NMPC_TERMINAL_SCALE,
          "terminal weight changed: regenerate solver");
  for (int i = 0; i < 16; ++i) require(qw[i] == GIMBALROTOR_NMPC_Q[i], "state weights changed: regenerate solver");
  for (int i = 0; i < 8; ++i) require(rw[i] == GIMBALROTOR_NMPC_R[i], "input weights changed: regenerate solver");
  require(n.getParam("state_timeout", state_timeout_) && std::isfinite(state_timeout_) && state_timeout_ > dt && state_timeout_ <= 0.1,
          "state_timeout must be between one cycle and 0.1 s");
  require(n.getParam("servo_timeout", servo_timeout_) && std::isfinite(servo_timeout_) && servo_timeout_ > dt && servo_timeout_ <= 0.1,
          "servo_timeout must be between one cycle and 0.1 s");
  require(n.getParam("solve_budget", solve_budget_) && std::isfinite(solve_budget_) && solve_budget_ > 0 && solve_budget_ < dt,
          "solve_budget must be shorter than one cycle");
  const auto tmin = values(n, "thrust_min", 4), tmax = values(n, "thrust_max", 4);
  const auto amin = values(n, "angle_min", 4), amax = values(n, "angle_max", 4);
  const auto tau = values(n, "servo_time_constants", 4);
  const auto vmax = values(n, "velocity_max", 3), wmax = values(n, "angular_velocity_max", 3);
  for (int i = 0; i < 3; ++i)
  {
    require(vmax[i] > 0 && wmax[i] > 0, "state limits must be positive");
    bounds_.state_lower[i] = -vmax[i]; bounds_.state_upper[i] = vmax[i];
    bounds_.state_lower[3+i] = -wmax[i]; bounds_.state_upper[3+i] = wmax[i];
  }
  ros::NodeHandle group(nh_, "servo_controller/gimbals");
  bool no_state = false, filter = false;
  group.param("no_real_state", no_state, false);
  group.param("filter_flag", filter, false);
  require(!no_state && !filter && !group.hasParam("state_sub_topic"), "requires unfiltered common servo/states feedback");
  std::set<int> indices;
  for (int i = 0; i < 4; ++i)
  {
    auto joint = model->getUrdfModel().getJoint("gimbal"+std::to_string(i+1));
    require(joint && joint->limits && joint->type == urdf::Joint::REVOLUTE &&
            std::abs(joint->axis.x-1) < 1e-9 && std::abs(joint->axis.y)+std::abs(joint->axis.z) < 1e-9,
            "quad gimbal axis must be local +X with finite URDF limits");
    require(tau[i] > 0 && tmin[i] >= 0 && tmin[i] >= min_thrust_ && tmax[i] > tmin[i] &&
            tmin[i] >= model->getThrustLowerLimit() && tmax[i] <= model->getThrustUpperLimit() &&
            amin[i] < amax[i] && amin[i] >= joint->limits->lower && amax[i] <= joint->limits->upper,
            "invalid actuator limits or servo time constant");
    tau_[i] = tau[i];
    require(std::isfinite(joint->limits->velocity) && joint->limits->velocity > 0, "invalid URDF servo velocity limit");
    reference_angle_rate_[i] = joint->limits->velocity;
    bounds_.lower[i] = tmin[i]; bounds_.upper[i] = tmax[i];
    bounds_.lower[4+i] = bounds_.state_lower[6+i] = amin[i];
    bounds_.upper[4+i] = bounds_.state_upper[6+i] = amax[i];
    ros::NodeHandle servo(group, "controller"+std::to_string(i+1));
    bool local_filter = false;
    servo.param("filter_flag", local_filter, false);
    require(!local_filter, "per-servo filtering is unsupported by the measured-angle contract");
    std::string name;
    require(servo.getParam("name", name) && name == "gimbal"+std::to_string(i+1) &&
            servo.getParam("id", servo_index_[i]) && servo_index_[i] >= 0 && servo_index_[i] < 4 &&
            indices.insert(servo_index_[i]).second, "invalid servo logical index/name mapping");
    auto parameter = [&](const char* key) {
      double value = 0;
      require(servo.getParam(key, value) || group.getParam(key, value), std::string("missing servo ")+key);
      require(std::isfinite(value), "nonfinite servo calibration");
      return value;
    };
    scale_[i] = parameter("angle_scale"); offset_[i] = parameter("zero_point_offset"); sign_[i] = parameter("angle_sgn");
    require(scale_[i] > 0 && std::abs(sign_[i]) == 1, "invalid servo scale/sign");
  }
  require(uav_model_ == spinal::UavInfo::DRONE && std::isfinite(max_pwm_) && std::isfinite(min_pwm_) &&
          min_pwm_ >= 0.5 && max_pwm_ > min_pwm_ && max_pwm_ <= 1 && !motor_info_.empty(), "invalid user MotorInfo PWM range");
  require(pwm_conversion_mode_ == spinal::MotorInfo::SQRT_MODE || pwm_conversion_mode_ == spinal::MotorInfo::POLYNOMINAL_MODE,
          "unknown thrust-to-PWM conversion");
  require(std::isfinite(m_f_rate_) && std::abs(m_f_rate_-model->getMFRate()) < 1e-8,
          "MotorInfo and URDF m_f_rate disagree");
  for (const auto& info : motor_info_)
  {
    require(std::isfinite(info.voltage) && info.voltage > 0 && std::isfinite(info.max_thrust) &&
            info.max_thrust >= *std::max_element(tmax.begin(), tmax.end()), "invalid motor reference voltage/thrust");
    for (double coefficient : info.polynominal) require(std::isfinite(coefficient), "nonfinite PWM coefficient");
  }
  std::string prefix;
  nhp_.param("tf_prefix", prefix, std::string(""));
  cog_frame_ = tf::resolve(prefix, "cog");
  std::string pose_topic;
  nh_.param("sensor_plugin/mocap/mocap_sub_name", pose_topic, std::string("pose"));
  thrust_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  servo_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  odom_sub_ = nh_.subscribe("uav/cog/odom", 1, &Controller::odomCallback, this);
  std::string servo_topic;
  nh_.param("servo_controller/state_sub_topic", servo_topic, std::string("servo/states"));
  servo_sub_ = nh_.subscribe(servo_topic, 1, &Controller::servoCallback, this);
  mocap_sub_ = nh_.subscribe(pose_topic, 1, &Controller::poseCallback, this);
  std::string imu_topic;
  nh_.param("sensor_plugin/imu/imu_topic_name", imu_topic, std::string("imu"));
  imu_sub_ = nh_.subscribe(imu_topic, 1, &Controller::imuCallback, this);
  ack_sub_ = nh_.subscribe("flight_config_ack", 1, &Controller::ackCallback, this);
  mode_client_ = nh_.serviceClient<std_srvs::SetBool>("set_attitude_control");
  solver_.reset(new Solver());
  ROS_WARN("NMPC: 40 Hz, horizon %.3f s, budget %.3f s; waiting for feedback and disarmed firmware handshake", N*dt, solve_budget_);
}

void Controller::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  if (msg->header.stamp <= odom_.header.stamp) return;
  odom_ = *msg;
  odom_receipt_ = ros::WallTime::now();
}

void Controller::poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  const auto q = orientation(msg->pose.orientation);
  if (msg->header.stamp <= pose_stamp_ || !q.coeffs().allFinite() || std::abs(q.norm()-1) > 0.01 ||
      !std::isfinite(msg->pose.position.x) || !std::isfinite(msg->pose.position.y) || !std::isfinite(msg->pose.position.z)) return;
  pose_stamp_ = msg->header.stamp;
  pose_receipt_ = ros::WallTime::now();
}

void Controller::imuCallback(const spinal::ImuConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  // Spinal's raw quaternion is xyzw, unlike the solver's wxyz state.
  const Eigen::Quaterniond q(msg->quaternion[3], msg->quaternion[0], msg->quaternion[1], msg->quaternion[2]);
  if (msg->stamp <= imu_stamp_ || !q.coeffs().allFinite() || std::abs(q.norm()-1) > 0.01) return;
  for (int i = 0; i < 3; ++i)
    if (!std::isfinite(msg->gyro[i]) || !std::isfinite(msg->acc[i]) || !std::isfinite(msg->mag[i])) return;
  imu_stamp_ = msg->stamp;
  imu_receipt_ = ros::WallTime::now();
}

void Controller::servoCallback(const spinal::ServoStatesConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  for (int i = 0; i < 4; ++i)
  {
    const auto count = std::count_if(msg->servos.begin(), msg->servos.end(), [&](const spinal::ServoState& s) { return s.index == servo_index_[i]; });
    if (count != 1) continue;
    const auto& s = *std::find_if(msg->servos.begin(), msg->servos.end(), [&](const spinal::ServoState& s) { return s.index == servo_index_[i]; });
    if (s.error || msg->stamp <= servo_stamp_[i]) continue;
    // Identical conversion to ServoBridge::SingleServoHandle; no command feedback.
    alpha_[i] = scale_[i]*sign_[i]*(double(s.angle)-offset_[i]);
    servo_stamp_[i] = msg->stamp;
    servo_receipt_[i] = ros::WallTime::now();
  }
}

void Controller::ackCallback(const std_msgs::UInt8ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  ack_ = msg->data;
  ack_receipt_ = ros::WallTime::now();
}

bool Controller::measuredState(State& x)
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  if (!fresh(odom_.header.stamp, odom_receipt_, state_timeout_) ||
      !fresh(pose_stamp_, pose_receipt_, state_timeout_) ||
      !fresh(imu_stamp_, imu_receipt_, state_timeout_) || odom_.header.stamp > imu_stamp_ ||
      (odom_.header.frame_id != "/world" && odom_.header.frame_id != "world") ||
      odom_.child_frame_id != cog_frame_ || estimator_->getUnhealthLevel() != 0) return false;
  // Master never sets experiment-mode CoG::Rot status; validate actual raw IMU
  // and its calibrated handler instead of waiting on an always-zero flag.
  if (estimator_->getImuHandlers().empty() ||
      estimator_->getImuHandler(0)->getStatus() != Status::ACTIVE) return false;
  for (int axis : {aerial_robot_estimation::State::X_BASE, aerial_robot_estimation::State::Y_BASE,
                  aerial_robot_estimation::State::Z_BASE})
    if (!estimator_->getStateStatus(axis, estimate_mode_)) return false;
  auto q = orientation(odom_.pose.pose.orientation);
  if (!q.coeffs().allFinite() || std::abs(q.norm()-1) > 0.01) return false;
  q.normalize();
  const auto& pos = odom_.pose.pose.position;
  const auto& v = odom_.twist.twist.linear;  // master publishes WORLD velocity here.
  const auto& w = odom_.twist.twist.angular; // master publishes COG angular velocity.
  x = {{pos.x, pos.y, pos.z, v.x, v.y, v.z, q.w(), q.x(), q.y(), q.z(), w.x, w.y, w.z, 0, 0, 0, 0}};
  for (int i = 0; i < 4; ++i)
  {
    if (!fresh(servo_stamp_[i], servo_receipt_[i], servo_timeout_)) return false;
    x[13+i] = alpha_[i];
  }
  if (!std::all_of(x.begin(), x.end(), [](double v) { return std::isfinite(v); })) return false;
  const int indices[] = {3, 4, 5, 10, 11, 12, 13, 14, 15, 16};
  for (int i = 0; i < 10; ++i)
    if (x[indices[i]] < bounds_.state_lower[i] || x[indices[i]] > bounds_.state_upper[i]) return false;
  double dot = 0;
  for (int i = 0; i < 4; ++i) dot += x[6+i]*previous_quaternion_[i];
  for (int i = 0; i < 4; ++i)
  {
    if (dot < 0) x[6+i] = -x[6+i];
    previous_quaternion_[i] = x[6+i];
  }
  return true;
}

Parameters Controller::physicalParameters()
{
  Parameters p{};
  p[99] = aerial_robot_estimation::G;
  require(robot_model_->getCogDesireOrientation<Eigen::Matrix3d>().isApprox(Eigen::Matrix3d::Identity(), 1e-8),
          "moving desired-CoG frame is unsupported; use FlightNav pose targets");
  p[0] = robot_model_->getMass();
  const Eigen::Matrix3d inertia = robot_model_->getInertia<Eigen::Matrix3d>();
  require(std::isfinite(p[0]) && p[0] > 0 && inertia.allFinite() && inertia.isApprox(inertia.transpose(), 1e-8) &&
          Eigen::LLT<Eigen::Matrix3d>(inertia).info() == Eigen::Success, "invalid mass/inertia");
  std::copy(inertia.data(), inertia.data()+9, p.begin()+1);
  const Eigen::Affine3d cog_inv = robot_model_->getCog<Eigen::Affine3d>().inverse();
  KDL::JntArray zero(robot_model_->getTree().getNrOfJoints());
  zero.data.setZero();
  for (int i = 0; i < 4; ++i)
  {
    const auto joint = robot_model_->getUrdfModel().getJoint("gimbal"+std::to_string(i+1));
    const Eigen::Affine3d pivot = cog_inv * robot_model_->forwardKinematics<Eigen::Affine3d>(joint->child_link_name, zero);
    const Eigen::Affine3d thrust = cog_inv * robot_model_->forwardKinematics<Eigen::Affine3d>("thrust"+std::to_string(i+1), zero);
    require((pivot.linear().transpose()*thrust.linear()).isApprox(Eigen::Matrix3d::Identity(), 1e-8),
            "thrust axes differ from quad gimbal zero frame");
    const Eigen::Matrix3d rotation = pivot.linear();
    const Eigen::Vector3d offset = rotation.transpose()*(thrust.translation()-pivot.translation());
    const int b = 10+17*i;
    std::copy(pivot.translation().data(), pivot.translation().data()+3, p.begin()+b);
    std::copy(rotation.data(), rotation.data()+9, p.begin()+b+3);
    std::copy(offset.data(), offset.data()+3, p.begin()+b+12);
    const int direction = robot_model_->getRotorDirection().at(i+1);
    require(std::abs(direction) == 1, "invalid rotor spin direction");
    p[b+15] = direction*robot_model_->getMFRate();
    p[b+16] = tau_[i];
  }
  require(std::all_of(p.begin(), p.begin()+NPHYS, [](double v) { return std::isfinite(v); }), "nonfinite geometry");
  return p;
}

std::vector<Reference> Controller::references(const State& x, const Parameters& p)
{
  Eigen::Vector3d pos = vector(navigator_->getTargetPos()), vel = vector(navigator_->getTargetVel());
  Eigen::Vector3d acc = vector(navigator_->getTargetAcc()), omega = vector(navigator_->getTargetOmega());
  Eigen::Vector3d angular_acc = vector(navigator_->getTargetAngAcc());
  const auto rpy = navigator_->getTargetRPY();
  require(pos.allFinite() && vel.allFinite() && acc.allFinite() && omega.allFinite() && angular_acc.allFinite() &&
          std::isfinite(rpy.x()) && std::isfinite(rpy.y()) && std::isfinite(rpy.z()), "nonfinite navigation reference");
  Eigen::Quaterniond q = Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX());
  const int navigation_state = navigator_->getNaviState();
  if (navigation_state != TAKEOFF_STATE && navigation_state != HOVER_STATE && navigation_state != LAND_STATE)
  {
    // Check a measured-pose hover OCP before arming, not an uninitialized goal.
    pos = Eigen::Map<const Eigen::Vector3d>(x.data());
    vel.setZero(); acc.setZero(); omega.setZero(); angular_acc.setZero();
    q = Eigen::Quaterniond(x[6], x[7], x[8], x[9]);
  }
  Eigen::Map<const Eigen::Matrix3d> inertia(p.data()+1);
  Eigen::Matrix<double, 6, 8> allocation;
  // Approximate allocation at the current rotor position, reference ONLY.
  for (int i = 0; i < 4; ++i)
  {
    int b = 10+17*i;
    Eigen::Map<const Eigen::Vector3d> pivot(p.data()+b), offset(p.data()+b+12);
    Eigen::Map<const Eigen::Matrix3d> rot(p.data()+b+3);
    const Eigen::Vector3d r = pivot + rot*Eigen::AngleAxisd(x[13+i], Eigen::Vector3d::UnitX())*offset;
    const Eigen::Vector3d horizontal = -rot.col(1), vertical = rot.col(2);
    allocation.block<3,1>(0, 2*i) = horizontal;
    allocation.block<3,1>(0, 2*i+1) = vertical;
    allocation.block<3,1>(3, 2*i) = r.cross(horizontal)+p[b+15]*horizontal;
    allocation.block<3,1>(3, 2*i+1) = r.cross(vertical)+p[b+15]*vertical;
  }
  Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix<double, 6, 8>> decomposition(allocation);
  require(decomposition.rank() == 6, "reference wrench map lacks rank six");
  std::vector<Reference> result(N+1);
  std::array<double, 4> previous = last_reference_alpha_;
  if (!have_reference_) std::copy(x.begin()+13, x.end(), previous.begin());
  for (int k = 0; k <= N; ++k)
  {
    const double t = k*GIMBALROTOR_NMPC_DT;
    auto& r = result[k];
    Eigen::Map<Eigen::Vector3d>(r.x.data()) = pos+t*vel+0.5*t*t*acc;
    Eigen::Map<Eigen::Vector3d>(r.x.data()+3) = vel+t*acc;
    r.x[6] = q.w(); r.x[7] = q.x(); r.x[8] = q.y(); r.x[9] = q.z();
    const Eigen::Vector3d w = omega+t*angular_acc;
    Eigen::Map<Eigen::Vector3d>(r.x.data()+10) = w;
    Eigen::Matrix<double, 6, 1> wrench;
    wrench.head<3>() = q.conjugate()*(p[0]*(acc+Eigen::Vector3d(0, 0, p[99])));
    wrench.tail<3>() = inertia*angular_acc+w.cross(inertia*w);
    const Eigen::Matrix<double, 8, 1> virtual_force = decomposition.solve(wrench);
    require(virtual_force.allFinite(), "nonfinite reference allocation");
    for (int i = 0; i < 4; ++i)
    {
      const double f = std::hypot(virtual_force[2*i], virtual_force[2*i+1]);
      double angle = f > 1e-8 ? std::atan2(virtual_force[2*i], virtual_force[2*i+1]) : previous[i];
      angle = previous[i]+std::remainder(angle-previous[i], 2*M_PI);
      r.thrust[i] = std::clamp(f, bounds_.lower[i], bounds_.upper[i]);
      const double step_limit = reference_angle_rate_[i]*GIMBALROTOR_NMPC_DT;
      r.x[13+i] = std::clamp(std::clamp(angle, previous[i]-step_limit, previous[i]+step_limit),
                              bounds_.lower[4+i], bounds_.upper[4+i]);
      if (std::abs(f-r.thrust[i]) > 1e-6 || std::abs(angle-r.x[13+i]) > 1e-6)
        ROS_WARN_THROTTLE(2, "NMPC reference allocation limited; requested wrench may be infeasible");
      previous[i] = r.x[13+i];
    }
    // Constant target acceleration prediction; navigator omega is target-body rate.
    const Eigen::Vector3d step = (omega+(t+0.5*GIMBALROTOR_NMPC_DT)*angular_acc)*GIMBALROTOR_NMPC_DT;
    if (step.norm() > 1e-12) q = (q*Eigen::Quaterniond(Eigen::AngleAxisd(step.norm(), step.normalized()))).normalized();
  }
  std::copy(result[0].x.begin()+13, result[0].x.end(), last_reference_alpha_.begin());
  have_reference_ = true;
  return result;
}

void Controller::sendStop()
{
  spinal::FlightConfigCmd cmd;
  cmd.cmd = spinal::FlightConfigCmd::ARM_OFF_CMD;
  navigator_->getFlightConfigPublisher().publish(cmd);
  // Do not rely on zero thrust being an ESC-off PWM. ARM_OFF is the stop action.
}

void Controller::stop(const std::string& reason)
{
  if (!fault_) ROS_ERROR_STREAM("NMPC fault latched; emergency motor stop: " << reason);
  fault_ = true;
  navigation_->setReady(false);
  navigator_->setNaviState(STOP_STATE);
  solver_->reset();
  sendStop();
}

bool Controller::update()
{
  const ros::WallTime begin = ros::WallTime::now();
  if (fault_) { sendStop(); return false; }
  const int state = navigator_->getNaviState();
  const bool flying = state == TAKEOFF_STATE || state == HOVER_STATE || state == LAND_STATE;
  if (mode_ready_ && !last_update_.isZero() && (begin-last_update_).toSec() > state_timeout_)
  { stop("missed control deadline"); return false; }
  last_update_ = begin;
  if (navigator_->getForceLandingFlag()) { stop("force landing requested; direct mode has no attitude fallback"); return false; }
  if (navigator_->getXyControlMode() != POS_CONTROL_MODE)
  { stop("MVP requires pose / pose-trajectory navigation mode"); return false; }
  State measured{};
  if (!measuredState(measured))
  {
    navigation_->setReady(false);
    if (state != ARM_OFF_STATE && state != STOP_STATE) stop("missing/stale/invalid CoG or servo measurement");
    else { sendStop(); ROS_WARN_THROTTLE(2, "NMPC waiting for valid CoG, mocap and all four measured servo states"); }
    return false;
  }
  try
  {
    if (!mode_ready_)
    {
      navigation_->setReady(false);
      if (state != ARM_OFF_STATE && state != STOP_STATE) { stop("direct mode was not acknowledged before arming"); return false; }
      if (mode_request_.isZero())
      {
        mode_request_ = begin;
        sendStop();
        ControlBase::activate(); // motor/UAV configuration only, while disarmed
        return false;
      }
      bool disarmed;
      {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        disarmed = ack_ == spinal::FlightConfigCmd::ARM_OFF_CMD && ack_receipt_ > mode_request_ &&
                   (begin-ack_receipt_).toSec() < state_timeout_;
      }
      if (!disarmed) { mode_request_ = ros::WallTime(); return false; }
      std_srvs::SetBool mode;
      mode.request.data = false;
      if (!mode_client_.exists() || !mode_client_.call(mode) || !mode.response.success ||
          mode.response.message != "nmpc_direct_thrust_v1")
      { stop("missing/incompatible set_attitude_control firmware acknowledgement"); return false; }
      mode_ready_ = true;
      solver_->reset();
      have_reference_ = false;
      ROS_WARN("NMPC direct mode acknowledged while disarmed; checking solver before enabling navigation arming");
      return false;
    }
    if (state == STOP_STATE)
    {
      sendStop();
      navigation_->setReady(false);
      mode_ready_ = false;
      mode_request_ = ros::WallTime();
      return false;
    }
    const auto parameters = physicalParameters();
    Input command{};
    const auto horizon = references(measured, parameters);
    const int status = solver_->solve(measured, parameters, bounds_, horizon, command);
    const double elapsed = (ros::WallTime::now()-begin).toSec();
    if (status || elapsed > solve_budget_ || !measuredState(measured))
    { stop("solver status " + std::to_string(status) + ", elapsed " + std::to_string(elapsed) + " s, or feedback expired"); return false; }
    // Navigation acknowledgements/stop callbacks can arrive during the solve.
    const int current_state = navigator_->getNaviState();
    if (navigator_->getForceLandingFlag()) { stop("force landing during solve"); return false; }
    if (flying && current_state != TAKEOFF_STATE && current_state != HOVER_STATE && current_state != LAND_STATE)
    { navigation_->setReady(false); sendStop(); return false; }
    if (current_state == STOP_STATE) { navigation_->setReady(false); sendStop(); return false; }
    navigation_->setReady(true);
    if (state == ARM_OFF_STATE || current_state == ARM_OFF_STATE) return false;
    spinal::FourAxisCommand thrust;
    thrust.base_thrust.resize(4, 0);
    if (flying)
    {
      sensor_msgs::JointState servo;
      servo.header.stamp = ros::Time::now();
      for (int i = 0; i < 4; ++i)
      {
        thrust.base_thrust[i] = command[i];
        servo.name.push_back("gimbal"+std::to_string(i+1));
        servo.position.push_back(command[4+i]);
      }
      servo_pub_.publish(servo);
    }
    // START / ARM_ON send zero physical thrust to maintain the direct watchdog.
    // Direct firmware maps zero to IDLE_DUTY, bypassing min-thrust conversion.
    thrust_pub_.publish(thrust);
    ROS_DEBUG_THROTTLE(1, "NMPC solve/update %.6f s", elapsed);
    return flying;
  }
  catch (const std::exception& error) { stop(error.what()); return false; }
}
}

PLUGINLIB_EXPORT_CLASS(gimbalrotor_nmpc::Controller, aerial_robot_control::ControlBase);
PLUGINLIB_EXPORT_CLASS(gimbalrotor_nmpc::Navigator, aerial_robot_navigation::BaseNavigator);
