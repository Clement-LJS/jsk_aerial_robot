#pragma once

#include <array>
#include <vector>
extern "C"
{
#include "acados_solver_gimbalrotor_tilt_quad_servo.h"
}
#include "nmpc_config.h"

namespace gimbalrotor_nmpc
{
constexpr int NX = 17, NU = 8, NP = 100, NPHYS = 78;
constexpr int N = GIMBALROTOR_TILT_QUAD_SERVO_N;
using State = std::array<double, NX>;
using Input = std::array<double, NU>;
using Parameters = std::array<double, NP>;
struct Reference
{
  State x{};
  std::array<double, 4> thrust{};
};
struct Bounds
{
  Input lower{}, upper{};
  // v_W(3), omega_G(3), alpha(4)
  std::array<double, 10> state_lower{}, state_upper{};
};

class Solver
{
public:
  Solver();
  ~Solver();
  Solver(const Solver&) = delete;
  Solver& operator=(const Solver&) = delete;
  void reset() { warm_ = false; }
  int solve(const State& state, Parameters parameters, const Bounds& bounds,
            const std::vector<Reference>& references, Input& command);
private:
  gimbalrotor_tilt_quad_servo_solver_capsule* capsule_ = nullptr;
  bool warm_ = false;
};
}
