#include <gimbalrotor/model/gimbalrotor_perching_model.h>
#include <Eigen/SVD>
#include <algorithm>
#include <memory>
#include <set>
#include <pluginlib/class_list_macros.h>

using namespace perching_geometry;
using perching_dynamics::MultilinkModel;

namespace
{
// Physical lock data only; no live model, navigator or FK cache ownership.
struct LockedGeometry
{
  KDL::Tree tree;
  std::map<std::string, uint32_t> joint_indices;
  KDL::JntArray held_joints;
  std::vector<std::string> coordinates, inertia_links;
  std::vector<Eigen::Vector3d> centers;
  std::vector<MultilinkModel::Link> links;
  GimbalrotorPerchingModel::Pose pivot;
  std::string body_name, marker_name;

  bool differential(const Eigen::VectorXd& q, MultilinkModel::GeometryState& out) const
  {
    if(q.size() != static_cast<int>(coordinates.size()) || !q.allFinite()) return false;
    KDL::JntArray joints = held_joints;
    for(int i = 1; i < q.size(); ++i) joints(joint_indices.at(coordinates[i])) = q[i];
    std::map<std::string, KDL::Frame> poses;
    if(!forwardKinematics(tree, joints, poses)) return false;
    Eigen::Isometry3d rotation = Eigen::Isometry3d::Identity();
    rotation.linear() = expSO3(Eigen::Vector3d::UnitY() * q[0]);
    const Eigen::Isometry3d world_root = pivot * rotation * fromKdl(poses.at(marker_name)).inverse();
    out.body = world_root * fromKdl(poses.at(body_name));
    out.centers.resize(links.size());
    for(std::size_t i = 0; i < links.size(); ++i)
    {
      out.centers[i] = world_root * fromKdl(poses.at(inertia_links[i]));
      out.centers[i].translation() = out.centers[i] * centers[i];
    }
    out.body_linear.resize(3, q.size());
    out.body_angular.resize(3, q.size());
    out.linear.assign(links.size(), Eigen::MatrixXd(3, q.size()));
    out.angular = out.linear;
    auto downstream = [&](const std::string& link, const std::string& joint)
    {
      auto segment = tree.getSegment(link);
      while(segment != tree.getRootSegment())
      {
        if(GetTreeElementSegment(segment->second).getJoint().getName() == joint) return true;
        segment = GetTreeElementParent(segment->second);
      }
      return false;
    };
    for(int j = 0; j < q.size(); ++j)
    {
      Eigen::Vector3d axis = pivot.linear().col(1), origin = pivot.translation();
      bool marker_moves = false;
      if(j > 0)
      {
        const auto segment = std::find_if(tree.getSegments().begin(), tree.getSegments().end(), [&](const auto& item)
        { return GetTreeElementSegment(item.second).getJoint().getName() == coordinates[j]; });
        if(segment == tree.getSegments().end()) return false;
        const auto& joint = GetTreeElementSegment(segment->second).getJoint();
        const auto parent = GetTreeElementParent(segment->second);
        const auto frame = world_root * fromKdl(poses.at(parent->first));
        const auto a = joint.twist(1).rot, p = joint.JointOrigin();
        axis = frame.linear() * Eigen::Vector3d(a.x(), a.y(), a.z());
        origin = frame * Eigen::Vector3d(p.x(), p.y(), p.z());
        marker_moves = downstream(marker_name, coordinates[j]);
      }
      // Fixing the marker subtracts its joint twist from every link's twist.
      const int body_sign = j == 0 ? 1 : int(downstream(body_name, coordinates[j])) - int(marker_moves);
      out.body_angular.col(j) = body_sign * axis;
      out.body_linear.col(j) = body_sign * axis.cross(out.body.translation() - origin);
      for(std::size_t i = 0; i < links.size(); ++i)
      {
        const int sign = j == 0 ? 1 : int(downstream(inertia_links[i], coordinates[j])) - int(marker_moves);
        out.angular[i].col(j) = sign * axis;
        out.linear[i].col(j) = sign * axis.cross(out.centers[i].translation() - origin);
      }
    }
    return true;
  }
};
}

bool GimbalrotorPerchingModel::configure(const std::vector<std::string>& active, std::string& reason)
{
  unlock();
  reason = "invalid marker, coordinate map, limits or inertia";
  const auto& tree = getTree();
  KDL::Chain chain;
  if(!tree.getChain(getBaselinkName(), "perching_pivot", chain)) return false;
  held_joints_ = getJointPositionsSnapshot();
  if(!held_joints_.data.allFinite()) return false;
  std::set<std::string> remaining;
  for(unsigned i = 0; i < chain.getNrOfSegments(); ++i)
  {
    const auto& joint = chain.getSegment(i).getJoint();
    if(joint.getType() != KDL::Joint::None) remaining.insert(joint.getName());
  }
  coordinates_ = {"alpha"};
  const int n = active.size() + 1;
  lower = Vector::Constant(n, -M_PI / 2);
  upper = -lower;
  lower[0] = -M_PI;
  upper[0] = M_PI;
  velocity = Vector::Constant(n, .1);
  acceleration = Vector::Constant(n, .1);
  velocity[0] = .1745329;
  for(const auto& name : active)
  {
    const auto joint = getUrdfModel().getJoint(name);
    if(!remaining.erase(name) || !getJointIndexMap().count(name) || !joint || joint->mimic ||
       (joint->type != urdf::Joint::REVOLUTE && joint->type != urdf::Joint::CONTINUOUS) ||
       !joint->limits || !std::isfinite(joint->limits->velocity) || joint->limits->velocity <= 0) return false;
    const int i = coordinates_.size();
    coordinates_.push_back(name);
    lower[i] = joint->type == urdf::Joint::REVOLUTE ? joint->limits->lower : -INFINITY;
    upper[i] = joint->type == urdf::Joint::REVOLUTE ? joint->limits->upper : INFINITY;
    velocity[i] = joint->limits->velocity;
    if(lower[i] >= upper[i]) return false;
  }
  if(!remaining.empty()) return false;
  links_.clear();
  inertia_links_.clear();
  centers_.clear();
  mass_ = 0;
  for(const auto& item : getExtraModuleMap())
    if(!getInertiaMap().count(item.second.getName()) || item.second.getJoint().getType() != KDL::Joint::None) return false;
  for(const auto& item : getInertiaMap())
  {
    if(!tree.getSegments().count(item.first)) return false;
    KDL::RigidBodyInertia total = KDL::RigidBodyInertia::Zero();
    accumulateLinkInertia(total, KDL::Frame::Identity(), item.first, item.second, getExtraModuleMap());
    MultilinkModel::Link link;
    link.mass = total.getMass();
    const auto inertia = total.RefPoint(total.getCOG()).getRotationalInertia();
    for(int r = 0; r < 3; ++r)
      for(int c = 0; c < 3; ++c) link.inertia(r, c) = inertia.data[3 * r + c];
    if(!std::isfinite(link.mass) || link.mass < 0 || !link.inertia.allFinite()) return false;
    links_.push_back(link);
    inertia_links_.push_back(item.first);
    centers_.emplace_back(total.getCOG().x(), total.getCOG().y(), total.getCOG().z());
    mass_ += link.mass;
  }
  if(mass_ <= 0) return false;
  cog_to_body_ = getCogDesireOrientation<Eigen::Matrix3d>().transpose();
  reason.clear();
  return true;
}

bool GimbalrotorPerchingModel::frames(const Vector& q, std::map<std::string, KDL::Frame>& poses) const
{
  if(q.size() != static_cast<int>(coordinates_.size()) || !q.allFinite()) return false;
  KDL::JntArray joints = held_joints_;
  for(int i = 1; i < q.size(); ++i) joints(getJointIndexMap().at(coordinates_[i])) = q[i];
  // The inherited FK touches KDL segment caches, but never the live joint state.
  std::lock_guard<std::mutex> lock(model_update_mutex_);
  poses = const_cast<GimbalrotorPerchingModel*>(this)->fullForwardKinematics(joints);
  poses[getTree().getRootSegment()->first] = KDL::Frame::Identity();
  return poses.count(getBaselinkName()) && poses.count("perching_pivot");
}

GimbalrotorPerchingModel::Pose GimbalrotorPerchingModel::bodyPivot() const
{
  const auto joints = getJointPositionsSnapshot();
  return fromKdl(forwardKinematics<KDL::Frame>(getBaselinkName(), joints).Inverse() *
                 forwardKinematics<KDL::Frame>("perching_pivot", joints));
}

bool GimbalrotorPerchingModel::bodyContact(const Vector& q, Pose& contact, Eigen::Vector3d& cog) const
{
  std::map<std::string, KDL::Frame> poses;
  if(!frames(q, poses)) return false;
  const Pose root_body = fromKdl(poses.at(getBaselinkName()));
  contact = root_body.inverse() * fromKdl(poses.at("perching_pivot"));
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  for(std::size_t i = 0; i < links_.size(); ++i)
    center += links_[i].mass * (fromKdl(poses.at(inertia_links_[i])) * centers_[i]);
  cog = root_body.inverse() * (center / mass_);
  return poseValid(contact) && cog.allFinite();
}

bool GimbalrotorPerchingModel::lock(const Pose& pivot, const KDL::JntArray& held_joints, const Eigen::Matrix3d& cog_to_body)
{
  unlock();
  if(coordinates_.empty() || links_.empty() || !poseValid(pivot) || !rotationValid(cog_to_body) ||
     held_joints.rows() != getTree().getNrOfJoints() || !held_joints.data.allFinite()) return false;
  pivot_ = pivot;
  held_joints_ = held_joints;
  cog_to_body_ = cog_to_body;
  locked_ = true;
  return true;
}

bool GimbalrotorPerchingModel::evaluate(const Vector& q, State& out) const
{
  std::map<std::string, KDL::Frame> poses;
  if(!locked_ || !frames(q, poses)) return false;
  Pose rotation = Pose::Identity(); rotation.linear() = expSO3(Eigen::Vector3d::UnitY() * q[0]);
  // Anchor the whole tree at the single external hinge. This also handles a
  // reversed body/contact chain; no assumed serial order or axis sign.
  const Pose world_root = pivot_ * rotation * fromKdl(poses.at("perching_pivot")).inverse();
  out.body = world_root * fromKdl(poses.at(getBaselinkName()));
  out.centers.resize(links_.size());
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  for(std::size_t i = 0; i < links_.size(); ++i)
  {
    out.centers[i] = world_root * fromKdl(poses.at(inertia_links_[i]));
    out.centers[i].translation() = out.centers[i] * centers_[i];
    center += links_[i].mass * out.centers[i].translation();
  }
  out.cog = Pose::Identity();
  out.cog.translation() = center / mass_;
  out.cog.linear() = out.body.linear() * cog_to_body_;
  return poseValid(out.body) && poseValid(out.cog);
}

bool GimbalrotorPerchingModel::estimate(const Pose& body, Vector& q, double& pivot_error, double& hinge_error) const
{
  Pose contact; Eigen::Vector3d cog;
  if(!locked_ || !poseValid(body) || !bodyContact(q, contact, cog)) return false;
  const Pose relative = pivot_.inverse() * body * contact;
  Eigen::Quaterniond rotation(relative.linear());
  const double angle = 2 * std::atan2(rotation.vec().dot(Eigen::Vector3d::UnitY()), rotation.w());
  q[0] += std::atan2(std::sin(angle - q[0]), std::cos(angle - q[0]));
  pivot_error = relative.translation().norm();
  hinge_error = logSO3(expSO3(-Eigen::Vector3d::UnitY() * q[0]) * relative.linear()).norm();
  return q.allFinite() && std::isfinite(pivot_error) && std::isfinite(hinge_error);
}

bool GimbalrotorPerchingModel::differential(const Vector& q, MultilinkModel::GeometryState& out) const
{
  State state;
  if(!evaluate(q, state)) return false;
  out.body = state.body;
  out.centers = state.centers;
  out.body_linear.resize(3, q.size());
  out.body_angular.resize(3, q.size());
  out.linear.assign(links_.size(), Eigen::MatrixXd(3, q.size()));
  out.angular = out.linear;
  // Differentiate the anchored candidate, not the cached free-root Jacobian.
  const double h = 1e-5;
  for(int j = 0; j < q.size(); ++j)
  {
    Vector plus = q, minus = q;
    plus[j] += h;
    minus[j] -= h;
    State p, m;
    if(!evaluate(plus, p) || !evaluate(minus, m)) return false;
    out.body_linear.col(j) = (p.body.translation() - m.body.translation()) / (2 * h);
    out.body_angular.col(j) = logSO3(p.body.linear() * m.body.linear().transpose()) / (2 * h);
    for(std::size_t i = 0; i < links_.size(); ++i)
    {
      out.linear[i].col(j) = (p.centers[i].translation() - m.centers[i].translation()) / (2 * h);
      out.angular[i].col(j) = logSO3(p.centers[i].linear() * m.centers[i].linear().transpose()) / (2 * h);
    }
  }
  return true;
}

bool GimbalrotorPerchingModel::jacobian(const Vector& q, Eigen::MatrixXd& angular) const
{
  MultilinkModel::GeometryState state;
  if(!differential(q, state)) return false;
  angular = state.body.linear().transpose() * state.body_angular;
  return angular.allFinite();
}

bool GimbalrotorPerchingModel::pitchTangent(const Vector& q, const std::vector<bool>& moving, Vector& tangent) const
{
  if(!locked_ || q.size() != 1 || moving.size() != 1 || !moving[0]) return false;
  tangent = Vector::Ones(1);
  return q.allFinite();
}

bool GimbalrotorPerchingModel::orient(const Vector& start, const Eigen::Matrix3d& target,
                                      const std::vector<bool>& moving, Vector& result) const
{
  State state;
  if(!rotationValid(target) || !evaluate(start, state)) return false;
  const Eigen::Vector3d axis_world = pivot_.linear().col(1);
  const Eigen::Vector3d rotation = logSO3(target * state.body.linear().transpose());
  if((rotation - axis_world * axis_world.dot(rotation)).norm() > 1e-5) return false;
  return pitch(start, axis_world.dot(rotation), moving, result);
}

bool GimbalrotorPerchingModel::pitch(const Vector& start, double angle, const std::vector<bool>& moving, Vector& result) const
{
  Vector tangent;
  if(!std::isfinite(angle) || !pitchTangent(start, moving, tangent)) return false;
  result = start + angle * tangent;
  return (result.array() >= lower.array()).all() && (result.array() <= upper.array()).all();
}

bool GimbalrotorPerchingModel::allowed(const Vector& q, double max_tilt, double min_pitch, double max_pitch) const
{
  State s;
  if(!evaluate(q, s) || (q.array() < lower.array() - 1e-8).any() || (q.array() > upper.array() + 1e-8).any()) return false;
  const double pitch = std::asin(std::clamp(-s.body.linear()(2, 0), -1.0, 1.0));
  return s.body.linear()(2, 2) >= std::cos(max_tilt) && pitch >= min_pitch && pitch <= max_pitch;
}

bool GimbalrotorPerchingModel::dynamics(MultilinkModel& result, std::string& reason) const
{
  if(!locked_) { reason = "no hinge lock"; return false; }
  // The caller holds the session mutex; protect live KDL caches while copying.
  auto snapshot = std::make_shared<LockedGeometry>();
  {
    std::lock_guard<std::mutex> lock(model_update_mutex_);
    snapshot->tree = getTree();
    snapshot->joint_indices = getJointIndexMap(); // The map uses KDL TreeElement::q_nr.
  }
  snapshot->held_joints = held_joints_;
  snapshot->coordinates = coordinates_;
  snapshot->inertia_links = inertia_links_;
  snapshot->centers = centers_;
  snapshot->links = links_;
  snapshot->pivot = pivot_;
  snapshot->body_name = getBaselinkName();
  snapshot->marker_name = "perching_pivot";
  const std::shared_ptr<const LockedGeometry> geometry = std::move(snapshot);
  return result.configure(coordinates_, links_, {}, reason,
      [geometry](const Vector& q, MultilinkModel::GeometryState& state) { return geometry->differential(q, state); });
}

bool GimbalrotorPerchingModel::effort(const Vector& q, const Vector& rate, const Vector& acceleration,
                                     const Vector& joint_effort, Eigen::Matrix<double, 6, 1>& wrench) const
{
  MultilinkModel model;
  MultilinkModel::Evaluation evaluation;
  MultilinkModel::GeometryState geometry;
  State state;
  std::string reason;
  if(acceleration.size() != q.size() || !acceleration.allFinite() || !evaluate(q, state) ||
     !differential(q, geometry) || !dynamics(model, reason) ||
     !model.evaluate(q, rate, joint_effort, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                     Eigen::Vector3d::Zero(), evaluation, reason)) return false;
  // Actuator force acts at the current body-attached CoG, not each link CoM.
  const Eigen::Vector3d offset = state.cog.translation() - state.body.translation();
  Eigen::MatrixXd work(q.size(), 6);
  work.leftCols(3) = geometry.body_linear.transpose() + geometry.body_angular.transpose() * aerial_robot_model::skew(offset);
  work.rightCols(3) = geometry.body_angular.transpose();
  // d(M qdot)/dt = M qddot + Mdot qdot; predicted_rate includes gravity and joint actuation.
  const double h = 1e-4;
  MultilinkModel::Evaluation plus, minus;
  if(!model.evaluate(q + h * rate, rate, Vector::Zero(q.size()), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                     Eigen::Vector3d::Zero(), plus, reason) ||
     !model.evaluate(q - h * rate, rate, Vector::Zero(q.size()), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                     Eigen::Vector3d::Zero(), minus, reason)) return false;
  const Vector required = evaluation.mass * acceleration + (plus.momentum - minus.momentum) / (2 * h) - evaluation.predicted_rate;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(work, Eigen::ComputeThinU | Eigen::ComputeThinV);
  svd.setThreshold(1e-6);
  wrench = svd.solve(required);
  return wrench.allFinite() && (work * wrench - required).norm() < 1e-5;
}

PLUGINLIB_EXPORT_CLASS(GimbalrotorPerchingModel, aerial_robot_model::RobotModel);
