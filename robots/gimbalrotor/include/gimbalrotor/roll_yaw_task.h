#pragma once
#include <gimbalrotor/roll_yaw_perching_kinematics.h>
#include <algorithm>
#include <cmath>

namespace roll_yaw
{
// Signed continuous Y twist, with a full swing residual. Quaternion hemisphere
// changes disappear through nearest-2pi unwrapping; a 180-degree swing is invalid.
inline bool taskPitch(const Mat& pre,const Mat& body,double previous,double& pitch,double& cross_track)
{
  if(!rotationValid(pre)||!rotationValid(body)||!std::isfinite(previous)) return false;
  Eigen::Quaterniond q(pre.transpose()*body); q.normalize();
  if(std::hypot(q.w(),q.y())<1e-8) return false;
  pitch=previous+std::remainder(2*std::atan2(q.y(),q.w())-previous,2*M_PI);
  cross_track=logSO3(expSO3(-Vec::UnitY()*pitch)*pre.transpose()*body).norm();
  return std::isfinite(pitch)&&std::isfinite(cross_track);
}
struct TaskProgress
{
  double raw=0, filtered=0, distance=0, cross_track=0, stamp=0;
  bool valid=false;
  void reset(double start) { *this=TaskProgress(); raw=filtered=start; }
  bool observe(const Mat& pre,const Mat& body,double time,double start,double end,double filter_time,double tolerance)
  {
    if(time<=stamp) return valid;
    double pitch,residual;
    if(!taskPitch(pre,body,raw,pitch,residual)) { valid=false; return false; }
    const double dt=time-stamp;
    raw=pitch; cross_track=residual;
    filtered=stamp>0?filtered+(1-std::exp(-dt/filter_time))*(raw-filtered):raw;
    stamp=time; valid=true;
    if(residual<=tolerance) {
      const double sign=end>=start?1:-1;
      // Require agreement of raw and filtered samples; reverse motion does not
      // rewind the nominal path. Completion still uses raw measured orientation.
      distance=std::max(distance,std::clamp(std::min(sign*(raw-start),sign*(filtered-start)),0.0,std::abs(end-start)));
    }
    return true;
  }
};
}
