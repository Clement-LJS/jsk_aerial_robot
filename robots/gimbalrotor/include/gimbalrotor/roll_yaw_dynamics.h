#pragma once
#include <gimbalrotor/roll_yaw_perching_kinematics.h>
#include <kdl/treefksolverpos_recursive.hpp>
#include <vector>

namespace roll_yaw
{
struct DynamicsTerms
{
  bool ok=false;
  Vec momentum=Vec::Zero(), drift=Vec::Zero(), pitch_map=Vec::Zero();
  Mat mass=Mat::Zero(); double kinetic=0, potential=0;
  std::string reason="no dynamics sample";
};
// Worker-owned copied model. The inertia map already aggregates fixed children;
// do not sum tree inertias a second time. Extra modules enter exactly once.
class ArticulatedDynamics
{
public:
  ArticulatedDynamics(Kinematics model,const KDL::Tree&,const std::map<std::string,KDL::RigidBodyInertia>&,
                      const std::map<std::string,KDL::Segment>&,std::string base,unsigned roll,unsigned yaw,Vec gravity);
  DynamicsTerms evaluate(const Vec& eta,const Vec& rate,const KDL::JntArray& joints,const KDL::JntArray& rates,
                         const Vec& force_world,const Vec& torque_cog_world,const Vec& servo,
                         double min_singular,const std::function<bool()>& cancelled);
private:
  struct Link { std::string name; Pose local; double mass; Mat inertia; };
  bool poses(const Vec&,const KDL::JntArray&,std::vector<Pose>&);
  bool energy(const Vec&,const Vec&,const KDL::JntArray&,const KDL::JntArray&,DynamicsTerms&);
  Kinematics model_; KDL::TreeFkSolverPos_recursive fk_; std::vector<Link> links_;
  std::string base_; unsigned roll_,yaw_; Vec gravity_;
};
class MomentumObserver
{
public:
  double gain=10, torque=0; Vec residual=Vec::Zero();
  bool update(const DynamicsTerms&,double stamp,double max_gap);
  void reset() { initialized_=false; stamp_=0; residual.setZero(); torque=0; }
private:
  bool initialized_=false; double stamp_=0; Vec initial_=Vec::Zero(),integral_=Vec::Zero();
};
}
