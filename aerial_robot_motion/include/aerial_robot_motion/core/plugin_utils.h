#pragma once
#include <aerial_robot_motion/core/motion_state.h>
#include <ros/node_handle.h>
#include <cmath>
#include <stdexcept>

namespace aerial_robot_motion
{
inline Eigen::VectorXd vectorParam(const ros::NodeHandle& nh, const std::string& name,
                                  int length, double fallback, bool nonnegative = true)
{
  std::vector<double> values;
  if (!nh.getParam(name, values))
  {
    if (nh.hasParam(name)) throw std::runtime_error(nh.resolveName(name) + " must be a numeric vector");
    values.assign(length, fallback);
  }
  if (static_cast<int>(values.size()) != length)
    throw std::runtime_error(nh.resolveName(name) + " has incorrect vector length");
  Eigen::VectorXd result(length);
  for (int i = 0; i < length; ++i)
  {
    if (!std::isfinite(values[i]) || (nonnegative && values[i] < 0))
      throw std::runtime_error(nh.resolveName(name) + " contains invalid values");
    result(i) = values[i];
  }
  return result;
}
inline double nonnegativeParam(const ros::NodeHandle& nh, const std::string& name, double fallback)
{
  double value; nh.param(name, value, fallback);
  if (!std::isfinite(value) || value < 0) throw std::runtime_error(nh.resolveName(name) + " must be finite and nonnegative");
  return value;
}
inline bool validStep(double dt) { return std::isfinite(dt) && dt > 0 && dt <= 1.0; }
}
