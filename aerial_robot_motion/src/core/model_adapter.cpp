#include <aerial_robot_motion/core/model_adapter.h>
#include <aerial_robot_model/model/transformable_aerial_robot_model.h>
#include <kdl/treejnttojacsolver.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <XmlRpcValue.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace aerial_robot_motion
{
namespace
{
Eigen::Isometry3d eigenFrame(const KDL::Frame& frame)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  for (int row = 0; row < 3; ++row)
  {
    result.translation()[row] = frame.p[row];
    for (int col = 0; col < 3; ++col) result.linear()(row, col) = frame.M(row, col);
  }
  return result;
}

Eigen::Vector3d vectorParameter(const ros::NodeHandle& nh, const std::string& name)
{
  std::vector<double> values;
  if (!nh.getParam(name, values))
  {
    if (nh.hasParam(name)) throw std::invalid_argument(name + " must be a numeric three-vector");
    return Eigen::Vector3d::Zero();
  }
  if (values.size() != 3) throw std::invalid_argument(name + " must have three entries");
  Eigen::Vector3d result(values[0], values[1], values[2]);
  if (!result.allFinite()) throw std::invalid_argument(name + " must be finite");
  return result;
}

Eigen::Isometry3d offsetParameter(const ros::NodeHandle& nh, const std::string& prefix)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = vectorParameter(nh, prefix + "_xyz");
  const Eigen::Vector3d rpy = vectorParameter(nh, prefix + "_rpy");
  result.linear() = (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
                     Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
                     Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
  return result;
}
}  // namespace

ModelAdapter::ModelAdapter() : loader_("aerial_robot_model", "aerial_robot_model::RobotModel") {}

void ModelAdapter::initialize(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
{
  std::string plugin;
  if (!private_nh.getParam("model_plugin", plugin) && !nh.getParam("robot_model_plugin_name", plugin))
    throw std::invalid_argument("Set robot_model_plugin_name in the robot namespace or ~model_plugin");
  // Existing model constructors resolve robot_description using the node namespace.
  std::string description;
  if (!nh.searchParam("robot_description", description))
    throw std::invalid_argument("robot_description is missing in the robot namespace");
  initializeModel(loader_.createInstance(plugin), private_nh);
}

void ModelAdapter::initializeModel(const boost::shared_ptr<aerial_robot_model::RobotModel>& model,
                                  const ros::NodeHandle& private_nh)
{
  model_ = model;
  transformable_model_ = boost::dynamic_pointer_cast<aerial_robot_model::transformable::RobotModel>(model_);
  info_ = ModelInfo{}; required_joints_.clear(); fixed_joints_.clear();
  selected_model_joint_indices_.clear();
  need_static_thrust_ = need_joint_torque_ = need_feasible_control_ = false;
  if (!model_ || model_->getTree().getNrOfSegments() == 0 || model_->getMass() < 0)
    throw std::runtime_error("robot model has no valid KDL tree");
  info_.root_link = model_->getRootFrameName();
  if (transformable_model_)
  {
    info_.joint_names = transformable_model_->getLinkJointNames();
    info_.joint_indices = transformable_model_->getLinkJointIndices();
    info_.supports_physical_constraints = true;
    info_.thrust_lower = model_->getThrustLowerLimit();
    info_.thrust_upper = model_->getThrustUpperLimit();
  }
  const int n = info_.joint_names.size();
  if (info_.joint_indices.size() != static_cast<size_t>(n))
    throw std::runtime_error("link joint name/index sizes disagree");
  info_.lower.resize(n);
  info_.upper.resize(n);
  info_.velocity.resize(n);
  info_.effort.resize(n);
  std::set<int> selected;
  const auto& model_joint_names = model_->getJointNames();
  for (int i = 0; i < n; ++i)
  {
    const auto joint = model_->getUrdfModel().getJoint(info_.joint_names[i]);
    if (!joint || !joint->limits || info_.joint_indices[i] < 0 ||
        info_.joint_indices[i] >= static_cast<int>(model_->getTree().getNrOfJoints()) ||
        !selected.insert(info_.joint_indices[i]).second)
      throw std::runtime_error("invalid or duplicate commanded joint " + info_.joint_names[i]);
    const bool continuous = joint->type == urdf::Joint::CONTINUOUS;
    info_.lower[i] = continuous ? -std::numeric_limits<double>::infinity() : joint->limits->lower;
    info_.upper[i] = continuous ? std::numeric_limits<double>::infinity() : joint->limits->upper;
    info_.velocity[i] = joint->limits->velocity;
    info_.effort[i] = joint->limits->effort;
    if (std::isnan(info_.lower[i]) || std::isnan(info_.upper[i]) || info_.lower[i] > info_.upper[i] ||
        !std::isfinite(info_.velocity[i]) || info_.velocity[i] <= 0)
      throw std::runtime_error("invalid mechanical limits for " + info_.joint_names[i]);
    const auto model_entry = std::find(model_joint_names.begin(), model_joint_names.end(), info_.joint_names[i]);
    if (model_entry == model_joint_names.end())
      throw std::runtime_error("commanded joint is absent from model Jacobian order: " + info_.joint_names[i]);
    selected_model_joint_indices_.push_back(
        static_cast<int>(std::distance(model_joint_names.begin(), model_entry)));
  }
  for (const auto& name : model_->getJointNames())
    required_joints_[name] = model_->getJointIndexMap().at(name);

  XmlRpc::XmlRpcValue fixed;
  if (private_nh.getParam("fixed_joint_positions", fixed))
  {
    if (fixed.getType() != XmlRpc::XmlRpcValue::TypeStruct)
      throw std::invalid_argument("fixed_joint_positions must map joint names to numeric positions");
    for (auto it = fixed.begin(); it != fixed.end(); ++it)
    {
      const auto required = required_joints_.find(it->first);
      if (required == required_joints_.end() || selected.count(required->second))
        throw std::invalid_argument("fixed_joint_positions must name a non-commanded model joint: " + it->first);
      double value;
      if (it->second.getType() == XmlRpc::XmlRpcValue::TypeDouble) value = static_cast<double>(it->second);
      else if (it->second.getType() == XmlRpc::XmlRpcValue::TypeInt) value = static_cast<int>(it->second);
      else throw std::invalid_argument("fixed joint position must be numeric");
      if (!std::isfinite(value)) throw std::invalid_argument("fixed joint position must be finite");
      fixed_joints_[required->second] = value;
      required_joints_.erase(required);
    }
  }
  private_nh.param("tool_frame", tool_frame_, model_->getBaselinkName());
  private_nh.param("contact_frame", contact_frame_, model_->getBaselinkName());
  for (const auto& name : {tool_frame_, contact_frame_, model_->getBaselinkName()})
    if (name != info_.root_link && model_->getTree().getSegments().count(name) == 0)
      throw std::invalid_argument("configured frame is absent from robot model: " + name);
  tool_offset_ = offsetParameter(private_nh, "tool_offset");
  contact_offset_ = offsetParameter(private_nh, "contact_offset");
}

void ModelAdapter::configurePhysicalData(bool static_thrust, bool joint_torque, bool feasible_control)
{
  if ((static_thrust || joint_torque || feasible_control) && !transformable_model_)
    throw std::invalid_argument("physical constraints require the generic transformable robot-model interface");
  need_static_thrust_ = static_thrust;
  need_joint_torque_ = joint_torque;
  need_feasible_control_ = feasible_control;
}

bool ModelAdapter::readJoints(const sensor_msgs::JointState& message, KDL::JntArray& full,
                              std::string& error) const
{
  if (message.name.size() != message.position.size() ||
      (!message.velocity.empty() && message.velocity.size() != message.name.size()))
  {
    error = "joint state name/position/velocity size mismatch";
    return false;
  }
  full.resize(model_->getTree().getNrOfJoints());
  KDL::SetToZero(full);
  for (const auto& entry : fixed_joints_) full(entry.first) = entry.second;
  std::set<std::string> seen;
  for (size_t i = 0; i < message.name.size(); ++i)
  {
    if (!seen.insert(message.name[i]).second || !std::isfinite(message.position[i]) ||
        (!message.velocity.empty() && !std::isfinite(message.velocity[i])))
    {
      error = "joint state contains duplicate names or nonfinite data";
      return false;
    }
    const auto entry = required_joints_.find(message.name[i]);
    if (entry != required_joints_.end()) full(entry->second) = message.position[i];
  }
  for (const auto& entry : required_joints_)
    if (!seen.count(entry.first))
    {
      error = "joint state missing required joint " + entry.first;
      return false;
    }
  return true;
}

void ModelAdapter::setCogDesiredOrientation(const Eigen::Matrix3d& r)
{
  if (!r.allFinite()) throw std::invalid_argument("nonfinite CoG desired orientation");
  model_->setCogDesireOrientation(KDL::Rotation(r(0, 0), r(0, 1), r(0, 2),
                                               r(1, 0), r(1, 1), r(1, 2),
                                               r(2, 0), r(2, 1), r(2, 2)));
}

MotionState ModelAdapter::measured(const Eigen::Isometry3d& world_T_baselink,
                                  const KDL::JntArray& joints, const ros::Time& stamp)
{
  MotionState result;
  result.stamp = stamp;
  result.full_joints = joints;
  const auto root_T_baselink = eigenFrame(model_->forwardKinematics<KDL::Frame>(model_->getBaselinkName(), joints));
  result.root = world_T_baselink * root_T_baselink.inverse();
  update(result);
  return result;
}

MotionState ModelAdapter::integrate(const MotionState& measured, const Eigen::VectorXd& velocity,
                                   double dt, const ros::Time& stamp)
{
  if (velocity.size() != info_.dimension() || !velocity.allFinite() || !std::isfinite(dt) || dt <= 0)
    throw std::invalid_argument("invalid generalized velocity or integration interval");
  MotionState result = measured;
  result.stamp = stamp;
  result.root.translation() += dt * velocity.head<3>();
  result.root.linear() = rotationExp(dt * velocity.segment<3>(3)) * measured.root.linear();
  for (size_t i = 0; i < info_.joint_indices.size(); ++i)
    result.full_joints(info_.joint_indices[i]) += dt * velocity[6 + i];
  update(result);
  return result;
}

void ModelAdapter::update(MotionState& state)
{
  model_->updateRobotModel(state.full_joints);
  state.joints.resize(info_.joint_names.size());
  for (size_t i = 0; i < info_.joint_indices.size(); ++i) state.joints[i] = state.full_joints(info_.joint_indices[i]);
  state.tool = framePose(state, tool_frame_, tool_offset_);
  state.contact = framePose(state, contact_frame_, contact_offset_);
  state.cog = state.root * eigenFrame(model_->getCog<KDL::Frame>());
  state.tool_jacobian = frameJacobian(state, tool_frame_, tool_offset_);
  state.contact_jacobian = frameJacobian(state, contact_frame_, contact_offset_);
  updatePhysical(state);
  if (!state.root.matrix().allFinite() || !state.tool.matrix().allFinite() ||
      !state.contact.matrix().allFinite() || !state.cog.matrix().allFinite() ||
      !state.tool_jacobian.allFinite() || !state.contact_jacobian.allFinite())
    throw std::runtime_error("robot model produced nonfinite kinematics");
}

Eigen::MatrixXd ModelAdapter::qpPhysicalJacobian(const MotionState& state,
                                                 const Eigen::MatrixXd& full) const
{
  const int model_dimension = 6 + model_->getJointNum();
  if ((full.cols() != model_dimension && full.cols() != info_.dimension()) ||
      full.rows() == 0 || !full.allFinite())
    return Eigen::MatrixXd{};
  Eigen::MatrixXd mapping = Eigen::MatrixXd::Zero(full.cols(), info_.dimension());
  // Transform world-frame root rates into the KDL-root axes used by the
  // transformable model's virtual six root columns.
  mapping.block<3, 3>(0, 0) = state.root.linear().transpose();
  mapping.block<3, 3>(3, 3) = state.root.linear().transpose();
  if (full.cols() != model_dimension)
  {
    mapping.bottomRightCorner(info_.joint_names.size(), info_.joint_names.size()).setIdentity();
  }
  else
  {
    for (size_t i = 0; i < selected_model_joint_indices_.size(); ++i)
    {
      const int row = 6 + selected_model_joint_indices_[i];
      if (row < 6 || row >= model_dimension) return Eigen::MatrixXd{};
      mapping(row, 6 + i) = 1.0;
    }
  }
  return full * mapping;
}

void ModelAdapter::updatePhysical(MotionState& state)
{
  state.physical = PhysicalState{};
  if (!need_static_thrust_ && !need_joint_torque_ && !need_feasible_control_) return;
  if (!transformable_model_) throw std::runtime_error("transformable physical model is unavailable");

  // These calls are deliberately centralized. Plugins consume only values in
  // QP order and never guess how model-internal/gimbal joint columns are laid out.
  // Calling the virtual aggregate lets derived transformable models apply their
  // own internal-joint coupling before the adapter performs the final mapping.
  transformable_model_->updateJacobians(state.full_joints, false);

  if (need_static_thrust_ || need_joint_torque_)
  {
    if (need_static_thrust_)
    {
      state.physical.static_thrust = model_->getStaticThrust();
      state.physical.static_thrust_jacobian =
          qpPhysicalJacobian(state, transformable_model_->getLambdaJacobian());
      state.physical.static_thrust_available =
          state.physical.static_thrust.size() > 0 && state.physical.static_thrust.allFinite() &&
          state.physical.static_thrust_jacobian.rows() == state.physical.static_thrust.size() &&
          state.physical.static_thrust_jacobian.cols() == info_.dimension() &&
          state.physical.static_thrust_jacobian.allFinite();
    }
  }

  if (need_joint_torque_)
  {
    const auto& full_torque = transformable_model_->getJointTorque();
    const auto full_jacobian = qpPhysicalJacobian(state, transformable_model_->getJointTorqueJacobian());
    state.physical.joint_torque.resize(info_.joint_names.size());
    state.physical.joint_torque_jacobian.resize(info_.joint_names.size(), info_.dimension());
    bool valid = full_torque.size() == model_->getJointNum() &&
                 full_jacobian.rows() == model_->getJointNum() &&
                 full_jacobian.cols() == info_.dimension();
    for (size_t i = 0; valid && i < selected_model_joint_indices_.size(); ++i)
    {
      const int row = selected_model_joint_indices_[i];
      if (row < 0 || row >= full_torque.size()) { valid = false; break; }
      state.physical.joint_torque[i] = full_torque[row];
      state.physical.joint_torque_jacobian.row(i) = full_jacobian.row(row);
    }
    state.physical.joint_torque_available = valid && state.physical.joint_torque.allFinite() &&
                                             state.physical.joint_torque_jacobian.allFinite();
  }

  if (need_feasible_control_)
  {
    state.physical.feasible_force_margin = transformable_model_->getApproxFeasibleControlFDists();
    state.physical.feasible_torque_margin = transformable_model_->getApproxFeasibleControlTDists();
    state.physical.feasible_force_jacobian =
        qpPhysicalJacobian(state, transformable_model_->getFeasibleControlFDistsJacobian());
    state.physical.feasible_torque_jacobian =
        qpPhysicalJacobian(state, transformable_model_->getFeasibleControlTDistsJacobian());
    state.physical.feasible_control_available =
        state.physical.feasible_force_margin.size() > 0 &&
        state.physical.feasible_torque_margin.size() > 0 &&
        state.physical.feasible_force_margin.allFinite() &&
        state.physical.feasible_torque_margin.allFinite() &&
        state.physical.feasible_force_jacobian.rows() == state.physical.feasible_force_margin.size() &&
        state.physical.feasible_torque_jacobian.rows() == state.physical.feasible_torque_margin.size() &&
        state.physical.feasible_force_jacobian.cols() == info_.dimension() &&
        state.physical.feasible_torque_jacobian.cols() == info_.dimension() &&
        state.physical.feasible_force_jacobian.allFinite() &&
        state.physical.feasible_torque_jacobian.allFinite();
  }
  if (need_static_thrust_ && !state.physical.static_thrust_available)
    throw std::runtime_error("transformable model returned invalid static-thrust data or Jacobian");
  if (need_joint_torque_ && !state.physical.joint_torque_available)
    throw std::runtime_error("transformable model returned invalid selected-joint torque data or Jacobian");
  if (need_feasible_control_ && !state.physical.feasible_control_available)
    throw std::runtime_error("transformable model returned invalid feasible-control margins or Jacobians");
}

Eigen::Isometry3d ModelAdapter::framePose(const MotionState& state, const std::string& segment,
                                        const Eigen::Isometry3d& offset) const
{
  if (segment == info_.root_link) return state.root * offset;
  KDL::TreeFkSolverPos_recursive fk(model_->getTree());
  KDL::Frame frame;
  if (fk.JntToCart(state.full_joints, frame, segment) < 0) throw std::runtime_error("FK failed for " + segment);
  return state.root * eigenFrame(frame) * offset;
}

Eigen::MatrixXd ModelAdapter::frameJacobian(const MotionState& state, const std::string& segment,
                                          const Eigen::Isometry3d& offset) const
{
  const Eigen::Isometry3d world_T_point = framePose(state, segment, offset);
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(6, info_.dimension());
  result.topLeftCorner<3, 3>().setIdentity();
  result.block<3, 3>(0, 3) = -skew(world_T_point.translation() - state.root.translation());
  result.block<3, 3>(3, 3).setIdentity();
  if (info_.joint_names.empty() || segment == info_.root_link) return result;
  KDL::TreeJntToJacSolver solver(model_->getTree());
  KDL::Jacobian joint_jacobian(model_->getTree().getNrOfJoints());
  if (solver.JntToJac(state.full_joints, joint_jacobian, segment) < 0)
    throw std::runtime_error("joint Jacobian failed for " + segment);
  const Eigen::Isometry3d root_T_segment = state.root.inverse() * framePose(state, segment, Eigen::Isometry3d::Identity());
  const Eigen::Vector3d shift = root_T_segment.linear() * offset.translation();
  joint_jacobian.changeRefPoint(KDL::Vector(shift.x(), shift.y(), shift.z()));
  for (size_t i = 0; i < info_.joint_indices.size(); ++i)
  {
    result.block<3, 1>(0, 6 + i) = state.root.linear() * joint_jacobian.data.block<3, 1>(0, info_.joint_indices[i]);
    result.block<3, 1>(3, 6 + i) = state.root.linear() * joint_jacobian.data.block<3, 1>(3, info_.joint_indices[i]);
  }
  return result;
}
}  // namespace aerial_robot_motion
