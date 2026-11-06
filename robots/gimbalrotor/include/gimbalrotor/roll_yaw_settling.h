#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <string>

namespace roll_yaw
{
struct SettlingStatistics
{
  bool complete=false;
  double observed=0, body_rms=0, joint_rms=0, body_mean=0, joint_mean=0;
  double body_current=0, joint_current=0;
  std::string status="collecting fresh window";

  bool qualifies(double body_limit,double joint_limit,double mean_limit) const
  {
    return complete&&std::isfinite(body_limit)&&body_limit>0&&
      std::isfinite(joint_limit)&&joint_limit>0&&std::isfinite(mean_limit)&&mean_limit>0&&
      body_rms<body_limit&&joint_rms<joint_limit&&body_mean<mean_limit&&joint_mean<mean_limit&&
      body_current<body_limit&&joint_current<joint_limit;
  }
};

// Executor-local history. No ROS/Eigen dependencies or planner-worker state.
// Body XYZ and active roll/yaw are separate vector groups; passive alpha is excluded.
class SettlingWindow
{
public:
  void reset() { samples_.clear(); window_=0; }

  SettlingStatistics observe(double now,const std::array<double,3>& body,
                            const std::array<double,2>& joint,double window,double max_gap)
  {
    SettlingStatistics out;
    if(!std::isfinite(now)||!std::isfinite(window)||window<=0||
       !std::isfinite(max_gap)||max_gap<=0) {
      reset(); out.status="invalid window clock/configuration"; return out;
    }
    Sample sample{now,{{body[0],body[1],body[2],joint[0],joint[1]}}};
    for(double x:sample.rate) if(!std::isfinite(x)) {
      reset(); out.status="invalid angular rates"; return out;
    }
    if(window_!=0&&window_!=window) {
      reset(); out.status="window length changed; collecting fresh window";
    }
    if(!samples_.empty()) {
      const double dt=now-samples_.back().time;
      if(dt<=0||dt>max_gap) {
        reset(); out.status=dt<=0?"non-increasing clock; collecting fresh window":
          "sampling gap; collecting fresh window";
      }
    }
    window_=window;
    if(samples_.size()>=4096) {
      reset(); out.status="settling sample capacity exceeded; collecting fresh window"; return out;
    }
    samples_.push_back(sample);
    const double begin=now-window;
    // Retain the sample just before the left edge for exact linear clipping.
    while(samples_.size()>1&&samples_[1].time<=begin) samples_.pop_front();
    out.observed=std::min(window,now-samples_.front().time);
    out.body_current=std::hypot(std::hypot(body[0],body[1]),body[2]);
    out.joint_current=std::hypot(joint[0],joint[1]);
    if(samples_.size()<2||samples_.front().time>begin) return out;

    std::array<double,5> integral{{}},energy{{}};
    for(std::size_t i=1;i<samples_.size();++i) {
      const auto& a=samples_[i-1]; const auto& b=samples_[i];
      const double left=std::max(begin,a.time), dt=b.time-left;
      const double fraction=(left-a.time)/(b.time-a.time);
      for(std::size_t j=0;j<integral.size();++j) {
        const double x=(1-fraction)*a.rate[j]+fraction*b.rate[j], y=b.rate[j];
        integral[j]+=(x+y)*.5*dt;
        // Exact integral of squared linearly interpolated rate, not trapezoidal energy.
        energy[j]+=(x*x+x*y+y*y)*dt/3;
      }
    }
    // Subtract timestamps before dividing: epoch-valued clocks have finite precision.
    const double duration=now-begin;
    if(!std::isfinite(duration)||duration<=0) {
      reset(); out.status="unrepresentable window duration"; return out;
    }
    out.body_rms=std::sqrt((energy[0]+energy[1]+energy[2])/duration);
    out.joint_rms=std::sqrt((energy[3]+energy[4])/duration);
    out.body_mean=std::hypot(std::hypot(integral[0],integral[1]),integral[2])/duration;
    out.joint_mean=std::hypot(integral[3],integral[4])/duration;
    for(double x:{out.body_rms,out.joint_rms,out.body_mean,out.joint_mean,
                  out.body_current,out.joint_current}) if(!std::isfinite(x)) {
      reset(); out.status="nonfinite window statistics"; return out;
    }
    out.complete=true; out.status="complete window";
    return out;
  }

private:
  struct Sample { double time; std::array<double,5> rate; };
  std::deque<Sample> samples_;
  double window_=0;
};
}
