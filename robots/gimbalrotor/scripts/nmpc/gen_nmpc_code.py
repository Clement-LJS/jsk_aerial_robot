#!/usr/bin/env python3
"""Generate only (never compile) the single four-tilt solver with acados v0.5.0.

All physical parameters are supplied by the runtime robot model, at every stage.
x = [p_W(3), v_W(3), q_WG(wxyz)(4), omega_G(3), alpha(4)]
u = [f_N(4), alpha_command_rad(4)]
p = [mass, I_G column-major(9), four rotor blocks(17 each), x_ref(17), f_ref(4), gravity]
rotor block = [pivot_G(3), R_GE column-major(9), offset_E(3), signed_drag, tau]
"""
import argparse
from pathlib import Path

import casadi as ca
import numpy as np
import yaml
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver


def multiply(a, b):
    return ca.vertcat(a[0]*b[0] - ca.dot(a[1:4], b[1:4]),
                     a[0]*b[1:4] + b[0]*a[1:4] + ca.cross(a[1:4], b[1:4]))


def rotation(q):
    # Normalized rotation for integration drift; qdot uses the wxyz state.
    q = q / ca.sqrt(ca.dot(q, q) + 1e-16)
    w, x, y, z = (q[i] for i in range(4))
    return ca.vertcat(ca.horzcat(1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
                      ca.horzcat(2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
                      ca.horzcat(2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)))


def generate(profile):
    if not (Path(__file__).resolve().parents[4]/".git").exists():
        raise ValueError("Run the generator from the source repository, not the installed share directory")
    cfg = yaml.safe_load(profile.read_text())["controller"]["nmpc"]
    dt, steps = float(cfg["sample_time"]), int(cfg["steps"])
    if dt != 0.025 or steps < 2:
        raise ValueError("This MVP uses the master's 40 Hz loop; steps must be >= 2")
    q_weight = np.asarray(cfg["state_weights"], dtype=float)
    r_weight = np.asarray(cfg["input_weights"], dtype=float)
    terminal_scale = float(cfg["terminal_scale"])
    if (q_weight.shape != (16,) or r_weight.shape != (8,)
            or not np.isfinite(np.r_[q_weight, r_weight, terminal_scale]).all()
            or min(np.r_[q_weight, r_weight, terminal_scale]) <= 0):
        raise ValueError("Expected positive finite weights: 16 state, 8 input, terminal scale")
    model = AcadosModel()
    model.name = "gimbalrotor_tilt_quad_servo"
    x = model.x = ca.SX.sym("x", 17)
    u = model.u = ca.SX.sym("u", 8)
    p = model.p = ca.SX.sym("p", 100)
    model.xdot = ca.SX.sym("xdot", 17)
    inertia = ca.reshape(p[1:10], 3, 3)
    force, torque, adot = ca.SX.zeros(3), ca.SX.zeros(3), []
    for i in range(4):
        b = 10 + 17*i
        pivot, rot, offset = p[b:b+3], ca.reshape(p[b+3:b+12], 3, 3), p[b+12:b+15]
        alpha = x[13+i]
        rx = ca.vertcat(ca.horzcat(1, 0, 0),
                        ca.horzcat(0, ca.cos(alpha), -ca.sin(alpha)),
                        ca.horzcat(0, ca.sin(alpha), ca.cos(alpha)))
        direction = rot @ rx @ ca.vertcat(0, 0, 1)
        fi = direction*u[i]
        ri = pivot + rot @ rx @ offset
        force += fi
        # Local master convention: signed_drag = rotor_direction * m_f_rate.
        torque += ca.cross(ri, fi) + p[b+15]*fi
        adot.append((u[4+i] - alpha)/p[b+16])
    model.f_expl_expr = ca.vertcat(
        x[3:6], rotation(x[6:10]) @ force/p[0] + ca.vertcat(0, 0, -p[99]),
        0.5*multiply(x[6:10], ca.vertcat(0, x[10:13])),
        ca.solve(inertia, torque-ca.cross(x[10:13], inertia @ x[10:13])), *adot)
    model.f_impl_expr = model.xdot-model.f_expl_expr
    ref = p[78:95]
    # q_ref^-1 * q: reference-body-frame error, sign-invariant quadratic cost.
    qe = multiply(ca.vertcat(ref[6], -ref[7:10]), x[6:10])
    err = ca.vertcat(x[0:6]-ref[0:6], qe[1:4], x[10:17]-ref[10:17])
    model.cost_y_expr = ca.vertcat(err, u[0:4]-p[95:99], u[4:8]-x[13:17])
    model.cost_y_expr_e = err
    ocp = AcadosOcp()
    ocp.model = model
    ocp.solver_options.N_horizon = steps
    ocp.solver_options.tf = steps*dt
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 1
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.print_level = 0
    ocp.cost.cost_type = ocp.cost.cost_type_e = "NONLINEAR_LS"
    ocp.cost.W = np.diag(np.r_[q_weight, r_weight])
    ocp.cost.W_e = np.diag(q_weight*terminal_scale)
    ocp.cost.yref = np.zeros(24)
    ocp.cost.yref_e = np.zeros(16)
    ocp.constraints.idxbu = np.arange(8)
    # Zero bounds are deliberate: runtime must replace them before every solve.
    ocp.constraints.lbu = ocp.constraints.ubu = np.zeros(8)
    ocp.constraints.idxbx = np.r_[3:6, 10:17]
    ocp.constraints.lbx = ocp.constraints.ubx = np.zeros(10)
    ocp.constraints.idxbx_e = ocp.constraints.idxbx.copy()
    ocp.constraints.lbx_e = ocp.constraints.ubx_e = np.zeros(10)
    initial = np.zeros(17)
    initial[6] = 1
    ocp.constraints.x0 = initial
    # Algebraic initialization only, NOT robot calibration. Host replaces all 100.
    dummy = np.zeros(100)
    dummy[0] = 1
    dummy[1:10] = np.eye(3).ravel(order="F")
    for i in range(4):
        b = 10+17*i
        dummy[b+3:b+12] = np.eye(3).ravel(order="F")
        dummy[b+16] = 1
    dummy[84] = 1
    ocp.parameter_values = dummy
    output = Path(__file__).resolve().parents[2]/"generated"/"nmpc"
    output.mkdir(parents=True, exist_ok=True)
    ocp.code_export_directory = str(output)
    AcadosOcpSolver.generate(ocp, json_file=str(output/"ocp.json"))
    (output/"nmpc_config.h").write_text(
        "#pragma once\n#define GIMBALROTOR_NMPC_DT " + repr(dt) + "\n"
        + "constexpr double GIMBALROTOR_NMPC_Q[] = {" + ",".join(map(str, q_weight)) + "};\n"
        + "constexpr double GIMBALROTOR_NMPC_R[] = {" + ",".join(map(str, r_weight)) + "};\n"
        + "constexpr double GIMBALROTOR_NMPC_TERMINAL_SCALE = " + repr(terminal_scale) + ";\n")
    print(f"Generated sources only in {output}; native compilation is a separate step.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("-m", choices=["GimbalrotorTiltQuadServoNMPC"], required=True)
    parser.add_argument("--profile", type=Path,
                        default=Path(__file__).resolve().parents[2]/"config/quad/Nmpc.yaml")
    generate(parser.parse_args().profile)
