#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <map>
#include <functional>
#include <string>

namespace roll_yaw
{
using Vec = Eigen::Vector3d;
using Mat = Eigen::Matrix3d;
using Pose = Eigen::Isometry3d;
Vec logSO3(const Mat& r);
Mat expSO3(const Vec& v);
bool rotationValid(const Mat& r);
bool poseValid(const Pose& p);
Pose fromKdl(const KDL::Frame& f);

// Inertia-weighted CoG reconstruction from a copied articulated model. relative
// is R_GB, exactly RobotModel::getCogDesireOrientation(), not principal axes.
std::function<bool(const Vec&,Pose&)> articulatedCogModel(
  KDL::Tree tree,KDL::JntArray joints,std::map<std::string,KDL::RigidBodyInertia> inertia,
  std::map<std::string,KDL::Segment> extra,std::string base,KDL::Rotation relative,
  unsigned roll_index,unsigned yaw_index);

struct Estimate
{
  double alpha=0, pivot_error=0, hinge_residual=0;
};
// B=base_link, C=contact link, P=calibrated physical hinge, G=controller CoG.
// The callback supplies articulated T_BG, including the model's CoG orientation
// convention (not an inertia eigenvector frame). It must own an immutable snapshot.
class Kinematics
{
public:
  bool configure(const KDL::Chain& body_contact, const std::string& roll,
                 const std::string& yaw, const Pose& contact_pivot, const Vec& axis,
                 std::function<bool(const Vec&, Pose&)> body_cog, std::string& error);
  bool bodyPivot(const Vec& eta, Pose& bp) const;
  bool lock(const Pose& measured_body, const Vec& measured_eta);
  bool evaluate(const Vec& eta, Pose& body, Pose& cog) const;
  bool jacobian(const Vec& eta, Mat& j) const;
  bool numericalJacobian(const Vec& eta, Mat& j, double h=1e-6) const;
  bool estimate(const Pose& body, const Vec& measured_eta, Estimate& result) const;
  const Pose& pivotLock() const { return lock_; }
  bool locked() const { return locked_; }
private:
  KDL::Chain chain_;
  Pose cp_=Pose::Identity(), lock_=Pose::Identity();
  Vec axis_=Vec::UnitY();
  std::function<bool(const Vec&, Pose&)> cog_;
  bool configured_=false, locked_=false;
};
}
