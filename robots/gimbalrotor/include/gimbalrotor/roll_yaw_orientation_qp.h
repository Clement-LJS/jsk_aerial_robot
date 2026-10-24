#pragma once
#include <gimbalrotor/roll_yaw_perching_kinematics.h>
namespace roll_yaw
{
struct Limits
{
  Vec lower=Vec::Constant(-1.57079632679), upper=Vec::Constant(1.57079632679);
  Vec velocity=Vec(.15,.1,.1), acceleration=Vec(.1,.1,.1);
  bool valid() const;
};
struct QpConfig
{
  Vec weight=Vec::Ones();
  double regularization=1e-8, smoothing=1e-6, gain=2, min_singular_value=.05;
  bool valid() const;
};
struct QpResult { bool ok=false; Vec u=Vec::Zero(); double residual=0, sigma_min=0; std::string reason; };
// Exact deterministic enumeration of the 27 lower/free/upper active sets.
QpResult solveBoxQp(const Mat& h,const Vec& g,const Vec& lower,const Vec& upper);
QpResult orientationStep(const Mat& j,const Mat& current,const Mat& desired,const Vec& omega_des_body,
                        const Vec& eta,const Vec& previous,double dt,const Limits&,const QpConfig&);
}
