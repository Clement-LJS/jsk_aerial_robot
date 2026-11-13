#include <gimbalrotor/model/gimbalrotor_roll_yaw_dynamics.h>
#include <array>

namespace perching_dynamics
{
bool rollYawModel(const roll_yaw::Kinematics& geometry, const KDL::Tree& tree,
                 const KDL::JntArray& joints, const std::map<std::string, KDL::RigidBodyInertia>& inertia,
                 const std::map<std::string, KDL::Segment>& extra, const std::string& body,
                 unsigned ri, unsigned yi, const std::string& roll, const std::string& yaw,
                 MultilinkModel& result, std::string& reason)
{
  result = MultilinkModel();
  reason = "missing locked roll-yaw geometry, joint mapping or inertias";
  if(!geometry.locked() || ri == yi || ri >= joints.rows() || yi >= joints.rows() ||
     !joints.data.allFinite() || inertia.empty())
    return false;
  for(const auto& module : extra)
    if(inertia.count(module.second.getName()) != 1)
    {
      reason = "extra module parent is absent from inertia map";
      return false;
    }
  std::vector<std::string> names;
  std::vector<KDL::Vector> centers;
  std::vector<MultilinkModel::Link> links;
  for(const auto& item : inertia)
  {
    KDL::RigidBodyInertia aggregate = KDL::RigidBodyInertia::Zero();
    perching_geometry::accumulateLinkInertia(aggregate, KDL::Frame::Identity(), item.first, item.second, extra);
    MultilinkModel::Link link;
    link.mass = aggregate.getMass();
    // KDL getRotationalInertia() is about its reference origin, NOT its CoM.
    const auto ic = aggregate.RefPoint(aggregate.getCOG()).getRotationalInertia();
    for(int r = 0; r < 3; ++r)
      for(int c = 0; c < 3; ++c)
        link.inertia(r, c) = ic.data[3 * r + c];
    links.push_back(link);
    names.push_back(item.first);
    centers.push_back(aggregate.getCOG());
  }
  // Flatten once at capture. No tree iterators, solver references or mutable
  // scratch survive the snapshot: copies and concurrent evaluations are safe.
  struct Node
  {
    KDL::Segment segment;
    std::size_t parent;
    double frozen;
    int coordinate;
    unsigned descendants;
  };
  std::vector<Node> nodes;
  std::map<std::string, std::size_t> indices;
  std::array<std::size_t, 2> active{};
  unsigned found = 0;
  bool valid = true;
  std::function<void(KDL::SegmentMap::const_iterator, std::size_t, unsigned)> append;
  append = [&](KDL::SegmentMap::const_iterator it, std::size_t parent, unsigned mask)
  {
    const auto& segment = GetTreeElementSegment(it->second);
    const auto& joint = segment.getJoint();
    const bool moving = joint.getType() != KDL::Joint::None;
    const unsigned index = GetTreeElementQNr(it->second);
    const int coordinate = moving && index == ri ? 1 : moving && index == yi ? 2 : 0;
    if(moving && index >= joints.rows())
    {
      valid = false;
      return;
    }
    if(coordinate)
    {
      if(joint.getName() != (coordinate == 1 ? roll : yaw) ||
         joint.twist(1).rot.Norm() < 1e-12)
        valid = false;
      mask |= 1u << (coordinate - 1);
      found |= 1u << (coordinate - 1);
      active[coordinate - 1] = nodes.size();
    }
    const std::size_t current = nodes.size();
    indices[it->first] = current;
    nodes.push_back({segment, parent, moving ? joints(index) : 0, coordinate, mask});
    for(const auto& child : GetTreeElementChildren(it->second))
      append(child, current, mask);
  };
  const auto root = tree.getRootSegment();
  indices[root->first] = 0;
  nodes.push_back({KDL::Segment(), 0, 0, 0, 0});
  for(const auto& child : GetTreeElementChildren(root->second))
    append(child, 0, 0);
  if(!valid || found != 3 || !indices.count(body) || nodes[indices.at(body)].descendants != 0 ||
     !(nodes[active[0]].descendants & 2))
  {
    reason = "roll-yaw adapter requires body upstream of yaw then roll with matching joint indices";
    return false;
  }
  const auto body_index = indices.at(body);
  std::vector<std::size_t> link_indices;
  for(const auto& name : names)
  {
    if(!indices.count(name))
    {
      reason = "inertia link is absent from captured KDL tree";
      return false;
    }
    link_indices.push_back(indices.at(name));
  }
  auto differential = [geometry, nodes, active, body_index, link_indices, centers]
      (const Eigen::VectorXd& q, MultilinkModel::GeometryState& out)
  {
    Eigen::Matrix3d body_angular;
    if(q.size() != 3 || !geometry.evaluateBody(q, out.body) || !geometry.jacobian(q, body_angular))
      return false;
    // One root-to-tip pass supplies every link and joint pose at the same q.
    std::vector<KDL::Frame> poses(nodes.size(), KDL::Frame::Identity());
    for(std::size_t i = 1; i < nodes.size(); ++i)
    {
      const auto& node = nodes[i];
      poses[i] = poses[node.parent] * node.segment.pose(node.coordinate ? q[node.coordinate] : node.frozen);
    }
    const Eigen::Isometry3d world_root = out.body * roll_yaw::fromKdl(poses[body_index].Inverse());
    std::array<Eigen::Vector3d, 3> axes, origins;
    axes[0] = out.body.linear() * body_angular.col(0);
    origins[0] = geometry.pivotLock().translation();
    for(int j = 1; j < 3; ++j)
    {
      const auto& node = nodes[active[j - 1]];
      const auto& joint = node.segment.getJoint();
      const Eigen::Isometry3d parent = world_root * roll_yaw::fromKdl(poses[node.parent]);
      const KDL::Vector axis = joint.twist(1).rot, origin = joint.JointOrigin();
      axes[j] = parent.linear() * Eigen::Vector3d(axis.x(), axis.y(), axis.z());
      origins[j] = parent * Eigen::Vector3d(origin.x(), origin.y(), origin.z());
      // Inversion of body-to-contact gives negative active axes; refuse an
      // unsupported chain rather than quietly applying a different actuation map.
      if((axes[j] + out.body.linear() * body_angular.col(j)).norm() > 1e-8)
        return false;
    }
    auto columns = [&](const Eigen::Vector3d& point, unsigned descendants,
                       Eigen::MatrixXd& linear, Eigen::MatrixXd& angular)
    {
      linear.resize(3, 3);
      angular.resize(3, 3);
      for(int j = 0; j < 3; ++j)
      {
        // A link downstream of an active joint is fixed by the contact under
        // that joint variation. Upstream links rotate with the inverted chain.
        const double sign = j == 0 ? 1 : (descendants & (1u << (j - 1))) ? 0 : -1;
        angular.col(j) = sign * axes[j];
        linear.col(j) = (sign * axes[j]).cross(point - origins[j]);
      }
    };
    columns(out.body.translation(), 0, out.body_linear, out.body_angular);
    out.centers.resize(link_indices.size());
    out.linear.resize(link_indices.size());
    out.angular.resize(link_indices.size());
    for(std::size_t i = 0; i < link_indices.size(); ++i)
    {
      const auto index = link_indices[i];
      out.centers[i] = world_root * roll_yaw::fromKdl(poses[index] * KDL::Frame(centers[i]));
      columns(out.centers[i].translation(), nodes[index].descendants, out.linear[i], out.angular[i]);
    }
    return true;
  };
  return result.configure({"alpha", roll, yaw}, std::move(links), {}, reason, std::move(differential));
}
} // namespace perching_dynamics
