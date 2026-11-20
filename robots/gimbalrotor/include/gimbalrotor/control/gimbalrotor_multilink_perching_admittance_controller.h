// -*- mode: c++ -*-
#pragma once
#include <gimbalrotor/control/gimbalrotor_perching_admittance_controller.h>
namespace aerial_robot_control
{
// Compatibility type for the existing plugin ID. All layouts inherit one loop.
class GimbalrotorMultilinkPerchingAdmittanceController : public GimbalrotorPerchingAdmittanceController {};
}
