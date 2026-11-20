#include <gimbalrotor/model/gimbalrotor_multilink_perching_model.h>
#include <gimbalrotor/gimbalrotor_perching.h>
#include <Eigen/SVD>
#include <set>

namespace perching_dynamics
{
using namespace perching_geometry;

bool SerialModel::configure(const urdf::ModelInterface& urdf, const KDL::Tree& tree, const KDL::JntArray& joints,
    const std::map<std::string, KDL::RigidBodyInertia>& inertias,
    const std::map<std::string, KDL::Segment>& extra, const std::string& body, const std::string& contact,
    const std::vector<std::string>& active, const Pose& cp, const Eigen::Vector3d& axis,
    const Eigen::Matrix3d& cog_to_body, std::string& reason)
{
  *this = SerialModel();
  reason = "invalid contact chain, coordinate map, limits or inertia";
  KDL::Chain chain;
  if(!tree.getChain(body, contact, chain) || !joints.data.allFinite() || !poseValid(cp) ||
     !axis.allFinite() || std::abs(axis.norm() - 1) > 1e-6 || !rotationValid(cog_to_body)) return false;
  std::set<std::string> remaining;
  for(unsigned i = 0; i < chain.getNrOfSegments(); ++i)
  {
    const auto& joint = chain.getSegment(i).getJoint();
    if(joint.getType() != KDL::Joint::None) remaining.insert(joint.getName());
  }
  const int n = active.size() + 1;
  coordinates_ = {"alpha"};
  lower = Vector::Constant(n, -M_PI / 2); upper = -lower;
  lower[0] = -M_PI; upper[0] = M_PI;
  velocity = Vector::Constant(n, .1); acceleration = Vector::Constant(n, .1);
  velocity[0] = .1745329;
  std::map<std::string, int> coordinates;
  for(const auto& name : active)
  {
    auto joint = urdf.getJoint(name);
    if(!remaining.erase(name) || !joint || joint->mimic ||
       (joint->type != urdf::Joint::REVOLUTE && joint->type != urdf::Joint::CONTINUOUS) ||
       !joint->limits || !std::isfinite(joint->limits->velocity) || joint->limits->velocity <= 0) return false;
    const int i = coordinates_.size();
    coordinates[name] = i;
    coordinates_.push_back(name);
    lower[i] = joint->type == urdf::Joint::REVOLUTE ? joint->limits->lower : -INFINITY;
    upper[i] = joint->type == urdf::Joint::REVOLUTE ? joint->limits->upper : INFINITY;
    velocity[i] = joint->limits->velocity;
    if(lower[i] >= upper[i]) return false;
  }
  // Every articulated edge is explicit, including actuators held by a task.
  if(!remaining.empty()) return false;
  std::map<std::string, std::size_t> indices;
  bool valid = true;
  std::function<void(KDL::SegmentMap::const_iterator, std::size_t)> append;
  append = [&](KDL::SegmentMap::const_iterator it, std::size_t parent)
  {
    const auto& segment = GetTreeElementSegment(it->second);
    const bool moving = segment.getJoint().getType() != KDL::Joint::None;
    const unsigned index = GetTreeElementQNr(it->second);
    if(moving && index >= joints.rows()) { valid = false; return; }
    const auto coordinate = coordinates.find(segment.getJoint().getName());
    const std::size_t current = nodes_.size();
    indices[it->first] = current;
    nodes_.push_back({segment, parent, coordinate == coordinates.end() ? -1 : coordinate->second,
                      moving ? joints(index) : 0});
    for(const auto& child : GetTreeElementChildren(it->second)) append(child, current);
  };
  const auto root = tree.getRootSegment();
  indices[root->first] = 0;
  nodes_.push_back({KDL::Segment(), 0, -1, 0});
  for(const auto& child : GetTreeElementChildren(root->second)) append(child, 0);
  if(!valid || !indices.count(body) || !indices.count(contact)) return false;
  descendants_.assign(nodes_.size(), std::vector<bool>(n, false));
  for(std::size_t i = 1; i < nodes_.size(); ++i)
  {
    descendants_[i] = descendants_[nodes_[i].parent];
    if(nodes_[i].coordinate > 0) descendants_[i][nodes_[i].coordinate] = true;
  }
  body_ = indices.at(body); contact_ = indices.at(contact);
  for(const auto& item : extra)
    if(!inertias.count(item.second.getName()) || item.second.getJoint().getType() != KDL::Joint::None) return false;
  for(const auto& item : inertias)
  {
    if(!indices.count(item.first)) return false;
    KDL::RigidBodyInertia total = KDL::RigidBodyInertia::Zero();
    accumulateLinkInertia(total, KDL::Frame::Identity(), item.first, item.second, extra);
    MultilinkModel::Link link;
    link.mass = total.getMass();
    const auto inertia = total.RefPoint(total.getCOG()).getRotationalInertia();
    for(int r = 0; r < 3; ++r)
      for(int c = 0; c < 3; ++c) link.inertia(r, c) = inertia.data[3 * r + c];
    if(!std::isfinite(link.mass) || link.mass < 0 || !link.inertia.allFinite()) return false;
    links_.push_back(link); inertia_nodes_.push_back(indices.at(item.first));
    centers_.emplace_back(total.getCOG().x(), total.getCOG().y(), total.getCOG().z());
    mass_ += link.mass;
  }
  if(mass_ <= 0) return false;
  contact_pivot_ = cp; axis_ = axis; cog_to_body_ = cog_to_body;
  reason.clear();
  return true;
}

bool SerialModel::frames(const Vector& q, std::vector<Pose>& poses) const
{
  if(q.size() != static_cast<int>(coordinates_.size()) || !q.allFinite() || nodes_.empty()) return false;
  poses.assign(nodes_.size(), Pose::Identity());
  for(std::size_t i = 1; i < nodes_.size(); ++i)
  {
    // KDL caches joint poses: use a local segment, never mutate the snapshot.
    auto node = nodes_[i];
    poses[i] = poses[node.parent] * fromKdl(node.segment.pose(node.coordinate < 0 ? node.held : q[node.coordinate]));
  }
  return true;
}

bool SerialModel::bodyContact(const Vector& q, Pose& contact, Eigen::Vector3d& cog) const
{
  std::vector<Pose> poses;
  if(!frames(q, poses)) return false;
  contact = poses[body_].inverse() * poses[contact_];
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  for(std::size_t i = 0; i < links_.size(); ++i) center += links_[i].mass * (poses[inertia_nodes_[i]] * centers_[i]);
  cog = poses[body_].inverse() * (center / mass_);
  return poseValid(contact) && cog.allFinite();
}

bool SerialModel::lock(const Pose& body, const Vector& q)
{
  Pose contact; Eigen::Vector3d cog;
  if(!poseValid(body) || !bodyContact(q, contact, cog)) return false;
  Pose rotation = Pose::Identity(); rotation.linear() = expSO3(-axis_ * q[0]);
  pivot_ = body * contact * contact_pivot_ * rotation;
  locked_ = true;
  return true;
}

bool SerialModel::evaluate(const Vector& q, State& out) const
{
  std::vector<Pose> poses;
  if(!locked_ || !frames(q, poses)) return false;
  Pose rotation = Pose::Identity(); rotation.linear() = expSO3(axis_ * q[0]);
  // Anchor the whole tree at the single external hinge. This also handles a
  // reversed body/contact chain; no assumed serial order or axis sign.
  const Pose world_root = pivot_ * rotation * contact_pivot_.inverse() * poses[contact_].inverse();
  out.body = world_root * poses[body_];
  out.centers.resize(links_.size());
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  for(std::size_t i = 0; i < links_.size(); ++i)
  {
    out.centers[i] = world_root * poses[inertia_nodes_[i]];
    out.centers[i].translation() = out.centers[i] * centers_[i];
    center += links_[i].mass * out.centers[i].translation();
  }
  out.cog = Pose::Identity();
  out.cog.translation() = center / mass_;
  out.cog.linear() = out.body.linear() * cog_to_body_;
  return poseValid(out.body) && poseValid(out.cog);
}

bool SerialModel::estimate(const Pose& body, Vector& q, double& pivot_error, double& hinge_error) const
{
  Pose contact; Eigen::Vector3d cog;
  if(!locked_ || !poseValid(body) || !bodyContact(q, contact, cog)) return false;
  const Pose relative = pivot_.inverse() * body * contact * contact_pivot_;
  Eigen::Quaterniond rotation(relative.linear());
  const double angle = 2 * std::atan2(rotation.vec().dot(axis_), rotation.w());
  q[0] += std::atan2(std::sin(angle - q[0]), std::cos(angle - q[0]));
  pivot_error = relative.translation().norm();
  hinge_error = logSO3(expSO3(-axis_ * q[0]) * relative.linear()).norm();
  return q.allFinite() && std::isfinite(pivot_error) && std::isfinite(hinge_error);
}

bool SerialModel::differential(const Vector& q, MultilinkModel::GeometryState& out) const
{
  State state;
  std::vector<Pose> poses;
  if(!evaluate(q, state) || !frames(q, poses)) return false;
  const Pose world_root = state.body * poses[body_].inverse();
  std::vector<Eigen::Vector3d> axes(q.size()), origins(q.size());
  axes[0] = pivot_.linear() * axis_; origins[0] = pivot_.translation();
  for(std::size_t i = 1; i < nodes_.size(); ++i)
  {
    const auto& node = nodes_[i];
    if(node.coordinate < 0) continue;
    const auto& joint = node.segment.getJoint();
    const auto axis = joint.twist(1).rot, origin = joint.JointOrigin();
    const Pose parent = world_root * poses[node.parent];
    axes[node.coordinate] = parent.linear() * Eigen::Vector3d(axis.x(), axis.y(), axis.z());
    origins[node.coordinate] = parent * Eigen::Vector3d(origin.x(), origin.y(), origin.z());
  }
  auto jacobian = [&](std::size_t node, const Eigen::Vector3d& point, Eigen::MatrixXd& linear, Eigen::MatrixXd& angular)
  {
    linear.resize(3, q.size()); angular.resize(3, q.size());
    for(int j = 0; j < q.size(); ++j)
    {
      const double sign = j == 0 ? 1 : int(descendants_[node][j]) - int(descendants_[contact_][j]);
      angular.col(j) = sign * axes[j];
      linear.col(j) = sign * axes[j].cross(point - origins[j]);
    }
  };
  out.body = state.body; out.centers = state.centers;
  jacobian(body_, state.body.translation(), out.body_linear, out.body_angular);
  out.linear.resize(links_.size()); out.angular.resize(links_.size());
  for(std::size_t i = 0; i < links_.size(); ++i)
    jacobian(inertia_nodes_[i], state.centers[i].translation(), out.linear[i], out.angular[i]);
  return true;
}

bool SerialModel::jacobian(const Vector& q, Eigen::MatrixXd& angular) const
{
  MultilinkModel::GeometryState state;
  if(!differential(q, state)) return false;
  angular = state.body.linear().transpose() * state.body_angular;
  return angular.allFinite();
}

bool SerialModel::pitchTangent(const Vector& q, const std::vector<bool>& moving, Vector& tangent) const
{
  Eigen::MatrixXd angular;
  if(moving.size() != coordinates_.size() || !jacobian(q, angular)) return false;
  std::vector<int> selected;
  for(int i = 0; i < q.size(); ++i) if(moving[i]) selected.push_back(i);
  if(selected.empty()) return false;
  Eigen::MatrixXd task(3, selected.size());
  for(std::size_t i = 0; i < selected.size(); ++i) task.col(i) = angular.col(selected[i]);
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(task, Eigen::ComputeThinU | Eigen::ComputeThinV);
  svd.setThreshold(1e-6);
  const Vector solution = svd.solve(Eigen::Vector3d::UnitY());
  // Rank is not a feasibility test: a single pitch axis is sufficient.
  if(!solution.allFinite() || (task * solution - Eigen::Vector3d::UnitY()).norm() > 1e-5) return false;
  tangent = Vector::Zero(q.size());
  for(std::size_t i = 0; i < selected.size(); ++i) tangent[selected[i]] = solution[i];
  return true;
}

bool SerialModel::orient(const Vector& start, const Eigen::Matrix3d& target, const std::vector<bool>& moving, Vector& result) const
{
  if(!rotationValid(target) || moving.size() != coordinates_.size()) return false;
  result = start;
  std::vector<int> selected;
  for(int i = 0; i < start.size(); ++i) if(moving[i]) selected.push_back(i);
  if(selected.empty()) return false;
  for(int iteration = 0; iteration < 40; ++iteration)
  {
    State state; Eigen::MatrixXd angular;
    if(!evaluate(result, state) || !jacobian(result, angular)) return false;
    const Eigen::Vector3d error = logSO3(state.body.linear().transpose() * target);
    if(error.norm() < 1e-6) return true;
    Eigen::MatrixXd task(3, selected.size());
    for(std::size_t j = 0; j < selected.size(); ++j) task.col(j) = angular.col(selected[j]);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(task, Eigen::ComputeThinU | Eigen::ComputeThinV);
    svd.setThreshold(1e-6);
    Vector delta = svd.solve(error);
    if(!delta.allFinite() || (task * delta - error).norm() > 1e-5) return false;
    if(delta.norm() > .1) delta *= .1 / delta.norm();
    for(std::size_t j = 0; j < selected.size(); ++j) result[selected[j]] += delta[j];
    if((result.array() < lower.array()).any() || (result.array() > upper.array()).any()) return false;
  }
  return false;
}

bool SerialModel::pitch(const Vector& start, double angle, const std::vector<bool>& moving, Vector& result) const
{
  if(!std::isfinite(angle) || std::abs(angle) > M_PI) return false;
  result = start;
  if(angle == 0) return start.size() == lower.size() && start.allFinite();
  // Integrate the very same tangent used for virtual work. This fixes the
  // null-space policy for parallel axes and preserves held coordinates exactly.
  const int steps = std::max(1, static_cast<int>(std::ceil(std::abs(angle) / .002)));
  const double h = angle / steps;
  for(int i = 0; i < steps; ++i)
  {
    Vector tangent, middle;
    if(!pitchTangent(result, moving, tangent) || !pitchTangent(result + .5 * h * tangent, moving, middle)) return false;
    result += h * middle;
    if((result.array() < lower.array()).any() || (result.array() > upper.array()).any()) return false;
  }
  State a, b;
  return evaluate(start, a) && evaluate(result, b) &&
      logSO3((a.body.linear() * expSO3(Eigen::Vector3d::UnitY() * angle)).transpose() * b.body.linear()).norm() < 1e-4;
}

bool SerialModel::allowed(const Vector& q, double max_tilt, double min_pitch, double max_pitch) const
{
  State s;
  if(!evaluate(q, s) || (q.array() < lower.array() - 1e-8).any() || (q.array() > upper.array() + 1e-8).any()) return false;
  const double pitch = std::asin(std::clamp(-s.body.linear()(2, 0), -1.0, 1.0));
  return s.body.linear()(2, 2) >= std::cos(max_tilt) && pitch >= min_pitch && pitch <= max_pitch;
}

bool SerialModel::dynamics(MultilinkModel& result, std::string& reason) const
{
  if(!locked_) { reason = "no hinge lock"; return false; }
  const auto snapshot = *this;
  return result.configure(coordinates_, links_, [snapshot](const Vector& q, MultilinkModel::Poses& centers, Pose& body)
  {
    State state;
    if(!snapshot.evaluate(q, state)) return false;
    centers = std::move(state.centers); body = state.body;
    return true;
  }, reason, [snapshot](const Vector& q, MultilinkModel::GeometryState& state) { return snapshot.differential(q, state); });
}
}
