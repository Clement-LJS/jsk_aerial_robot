// -*- mode: c++ -*-
#pragma once
#include <gimbalrotor/gimbalrotor_perching.h>
#include <gimbalrotor/model/gimbalrotor_multilink_perching_model.h>

namespace perching_dynamics
{
// Locked roll-yaw adapter only: q = [alpha, roll, yaw]. Omitted joints remain
// captured; the navigator must enforce their existing position/rate policy.
bool rollYawModel(const roll_yaw::Kinematics& geometry, const KDL::Tree& tree,
                 const KDL::JntArray& joints, const std::map<std::string, KDL::RigidBodyInertia>& inertia,
                 const std::map<std::string, KDL::Segment>& extra, const std::string& body,
                 unsigned roll_index, unsigned yaw_index, const std::string& roll, const std::string& yaw,
                 MultilinkModel& result, std::string& reason);
} // namespace perching_dynamics
