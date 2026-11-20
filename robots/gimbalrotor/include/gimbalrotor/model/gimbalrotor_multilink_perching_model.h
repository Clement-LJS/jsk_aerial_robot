// -*- mode: c++ -*-
#pragma once
#include <gimbalrotor/model/gimbalrotor_perching_model.h>

class GimbalrotorMultilinkPerchingModel : public GimbalrotorPerchingModel
{
public:
  using GimbalrotorPerchingModel::GimbalrotorPerchingModel;
  bool configure(const std::vector<std::string>& active, std::string& reason) override;
  bool pitchTangent(const Vector&, const std::vector<bool>&, Vector&) const override;
  bool orient(const Vector&, const Eigen::Matrix3d&, const std::vector<bool>&, Vector&) const override;
  bool pitch(const Vector&, double, const std::vector<bool>&, Vector&) const override;
};
