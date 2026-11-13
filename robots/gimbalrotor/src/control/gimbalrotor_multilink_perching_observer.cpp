#include <gimbalrotor/control/gimbalrotor_multilink_perching_observer.h>
#include <cmath>

namespace perching_dynamics
{
void MomentumObserver::reset(const std::string& reason)
{
  initial_.resize(0);
  integral_.resize(0);
  residual_.resize(0);
  stamp_ = 0;
  ready_ = false;
  reason_ = reason;
}

bool MomentumObserver::observe(const Eigen::VectorXd& p, const Eigen::VectorXd& b,
                               double stamp, double gain, double max_dt)
{
  if(p.size() == 0 || p.size() != b.size() || !p.allFinite() || !b.allFinite() ||
     !std::isfinite(stamp) || stamp <= 0 || !std::isfinite(gain) || gain <= 0 ||
     !std::isfinite(max_dt) || max_dt <= 0)
  {
    reset("invalid momentum observer input/gain");
    return false;
  }
  if(stamp_ == 0)
  {
    initial_ = p;
    integral_ = residual_ = Eigen::VectorXd::Zero(p.size());
    stamp_ = stamp;
    ready_ = false;
    reason_ = "rebased; need an advancing synchronized sample";
    return false;
  }
  const double dt = stamp - stamp_;
  if(dt <= 0 || dt > max_dt || p.size() != initial_.size())
  {
    reset("invalid observer dt/dimension; rebase required");
    return false;
  }
  residual_ = (gain / (1 + gain * dt)) * (p - initial_ - integral_ - dt * b);
  integral_ += dt * (b + residual_);
  if(!residual_.allFinite() || !integral_.allFinite())
  {
    reset("nonfinite momentum residual");
    return false;
  }
  stamp_ = stamp;
  ready_ = true;
  reason_ = "ready: generalized external load including unmodelled loads";
  return true;
}
} // namespace perching_dynamics
