#include <gimbalrotor/control/nmpc_solver.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace gimbalrotor_nmpc
{
static_assert(GIMBALROTOR_TILT_QUAD_SERVO_NX == NX, "Regenerate the 17-state solver");
static_assert(GIMBALROTOR_TILT_QUAD_SERVO_NU == NU, "Regenerate the 8-input solver");
static_assert(GIMBALROTOR_TILT_QUAD_SERVO_NP == NP, "Physical/reference parameter layout mismatch");
static_assert(GIMBALROTOR_TILT_QUAD_SERVO_NY == 24, "Cost layout mismatch");

Solver::Solver()
{
  capsule_ = gimbalrotor_tilt_quad_servo_acados_create_capsule();
  if (!capsule_) throw std::runtime_error("Cannot allocate NMPC capsule");
  if (gimbalrotor_tilt_quad_servo_acados_create(capsule_))
  {
    gimbalrotor_tilt_quad_servo_acados_free_capsule(capsule_);
    capsule_ = nullptr;
    throw std::runtime_error("Cannot create NMPC solver");
  }
}

Solver::~Solver()
{
  if (capsule_)
  {
    gimbalrotor_tilt_quad_servo_acados_free(capsule_);
    gimbalrotor_tilt_quad_servo_acados_free_capsule(capsule_);
  }
}

int Solver::solve(const State& state, Parameters parameters, const Bounds& bounds,
                  const std::vector<Reference>& references, Input& command)
{
  command.fill(0);
  if (references.size() != N+1) return -1;
  auto config = gimbalrotor_tilt_quad_servo_acados_get_nlp_config(capsule_);
  auto dims = gimbalrotor_tilt_quad_servo_acados_get_nlp_dims(capsule_);
  auto in = gimbalrotor_tilt_quad_servo_acados_get_nlp_in(capsule_);
  auto out = gimbalrotor_tilt_quad_servo_acados_get_nlp_out(capsule_);
  auto set = [&](int k, const char* field, const double* value) {
    // v0.5.0 takes nlp_out as well as nlp_in here.
    return ocp_nlp_constraints_model_set(config, dims, in, out, k, field,
                                          const_cast<double*>(value));
  };
  if (!warm_) gimbalrotor_tilt_quad_servo_acados_reset(capsule_, 1);
  if (set(0, "lbx", state.data()) || set(0, "ubx", state.data())) return -2;
  for (int k = 0; k <= N; ++k)
  {
    std::copy(references[k].x.begin(), references[k].x.end(), parameters.begin()+78);
    std::copy(references[k].thrust.begin(), references[k].thrust.end(), parameters.begin()+95);
    if (!std::all_of(parameters.begin(), parameters.end(), [](double v) { return std::isfinite(v); }))
      return -3;
    if (gimbalrotor_tilt_quad_servo_acados_update_params(capsule_, k, parameters.data(), NP)) return -4;
    if (k && (set(k, "lbx", bounds.state_lower.data()) || set(k, "ubx", bounds.state_upper.data()))) return -5;
    if (k < N && (set(k, "lbu", bounds.lower.data()) || set(k, "ubu", bounds.upper.data()))) return -6;

    State guess = state;
    Input input{};
    if (warm_)
      ocp_nlp_out_get(config, dims, out, std::min(k+1, N), "x", guess.data());
    if (k == 0) guess = state;
    // acados v0.5.0 out_set also requires nlp_in.
    ocp_nlp_out_set(config, dims, out, in, k, "x", guess.data());
    if (k < N)
    {
      if (warm_) ocp_nlp_out_get(config, dims, out, std::min(k+1, N-1), "u", input.data());
      else
      {
        std::copy(references[k].thrust.begin(), references[k].thrust.end(), input.begin());
        std::copy(state.begin()+13, state.end(), input.begin()+4);
      }
      ocp_nlp_out_set(config, dims, out, in, k, "u", input.data());
    }
  }
  int status = gimbalrotor_tilt_quad_servo_acados_solve(capsule_);
  warm_ = status == 0;
  if (status) return status;
  // SQP_RTI success alone does not certify finite/feasible predicted states.
  const int bounded_indices[] = {3, 4, 5, 10, 11, 12, 13, 14, 15, 16};
  for (int k = 0; k <= N; ++k)
  {
    State predicted{};
    ocp_nlp_out_get(config, dims, out, k, "x", predicted.data());
    double norm_squared = 0;
    for (int i = 6; i < 10; ++i) norm_squared += predicted[i]*predicted[i];
    if (!std::all_of(predicted.begin(), predicted.end(), [](double v) { return std::isfinite(v); }) ||
        std::abs(norm_squared-1) > 0.01) { warm_ = false; return -8; }
    for (int i = 0; i < 10; ++i)
      if (predicted[bounded_indices[i]] < bounds.state_lower[i]-1e-5 ||
          predicted[bounded_indices[i]] > bounds.state_upper[i]+1e-5) { warm_ = false; return -9; }
  }
  ocp_nlp_out_get(config, dims, out, 0, "u", command.data());
  for (int i = 0; i < NU; ++i)
    if (!std::isfinite(command[i]) || command[i] < bounds.lower[i]-1e-6 || command[i] > bounds.upper[i]+1e-6)
    {
      warm_ = false;
      command.fill(0);
      return -7;
    }
  for (int i = 0; i < NU; ++i)
    command[i] = std::clamp(command[i], bounds.lower[i], bounds.upper[i]);
  return 0;
}
}
