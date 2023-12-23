import casadi
import numpy as np
import os
import shutil
from acados_template import (
    AcadosModel,
    AcadosOcp,
    AcadosOcpSolver,
    AcadosSim,
    AcadosSimSolver
)

# diff-drive model
#
#                front
#              ^ x
#      | vl    |    vr |
# <- y |------ d ------|
#      |               |
#
d = 0.4  # distance between 2 wheels, meters
max_wheel_linear_vel = 1  # vl, vr, meters/sec
max_linear_acc = 5
max_angular_acc = 12
predict_time_horizon = 1.0
predict_node_num = 20
sim_dt = 0.01  # 100 Hz
mat_W = np.diag([5, 5, 1, 1, 3, 0.001, 0.001])
mat_We = np.diag([2, 2, 1, 1, 1.5])

# working directory = ego_planner/scripts
code_export_dir = os.path.join("../include", "acados_export_diffdrive")

# assumes acados is installed in the conda env
os.environ["ACADOS_SOURCE_DIR"] = os.path.join(os.environ["CONDA_PREFIX"], "acados")
acados_include_path = os.path.join(os.environ["CONDA_PREFIX"], "include")
acados_lib_path = os.path.join(os.environ["CONDA_PREFIX"], "lib")


# this function calculates the nonlinear error from states x and reference p
# the error vector has dim = 5, we want it to converge to zero
def nonlinear_error(x: casadi.SX, p: casadi.SX) -> casadi.SX:
    w_p_x = x[0]
    w_p_y = x[1]
    w_v_x = x[2]
    w_v_y = x[3]
    theta = x[4]

    ref_w_p_x = p[0]
    ref_w_p_y = p[1]
    ref_w_v_x = p[2]
    ref_w_v_y = p[3]
    ref_theta = p[4]

    error = casadi.vertcat(
        w_p_x - ref_w_p_x,
        w_p_y - ref_w_p_y,
        w_v_x - ref_w_v_x,
        w_v_y - ref_w_v_y,
        casadi.sin(theta / 2) * casadi.cos(ref_theta / 2) - casadi.cos(theta / 2) * casadi.sin(ref_theta / 2)
    )
    return error


# define robot dynamics for solving ocp
def create_diffdrive_model() -> AcadosModel:
    model = AcadosModel()
    model.name = "diffdrive"

    # states
    w_p_x = casadi.SX.sym("w_p_x")
    w_p_y = casadi.SX.sym("w_p_y")
    w_v_x = casadi.SX.sym("w_v_x")
    w_v_y = casadi.SX.sym("w_v_y")
    theta = casadi.SX.sym("theta")
    b_v = casadi.SX.sym("b_v")
    omega = casadi.SX.sym("omega")
    model.x = casadi.vertcat(w_p_x, w_p_y, w_v_x, w_v_y, theta, b_v, omega)

    # controls
    b_v_dot = casadi.SX.sym("b_v_dot")
    omega_dot = casadi.SX.sym("omega_dot")
    model.u = casadi.vertcat(b_v_dot, omega_dot)

    # dynamics
    model.f_expl_expr = casadi.vertcat(
        b_v * casadi.cos(theta),
        b_v * casadi.sin(theta),
        b_v_dot * casadi.cos(theta) - b_v * omega * casadi.sin(theta),
        b_v_dot * casadi.sin(theta) + b_v * omega * casadi.cos(theta),
        omega,
        b_v_dot,
        omega_dot,
    )

    # constrained variables
    model.con_h_expr = casadi.vertcat(b_v, b_v - d / 2 * omega, b_v + d / 2 * omega)
    model.con_h_expr_e = model.con_h_expr

    # state reference as parameters
    ref_w_p_x = casadi.SX.sym("ref_w_p_x")
    ref_w_p_y = casadi.SX.sym("ref_w_p_y")
    ref_w_v_x = casadi.SX.sym("ref_w_v_x")
    ref_w_v_y = casadi.SX.sym("ref_w_v_y")
    ref_theta = casadi.SX.sym("ref_theta")
    model.p = casadi.vertcat(ref_w_p_x, ref_w_p_y, ref_w_v_x, ref_w_v_y, ref_theta)

    # cost expression, y_ref and y_ref_e in acados is always set as zero
    model.cost_y_expr = casadi.vertcat(nonlinear_error(model.x, model.p), model.u)
    model.cost_y_expr_e = nonlinear_error(model.x, model.p)

    return model


# define the optimal control problem
def define_ocp() -> AcadosOcp:
    ocp = AcadosOcp()
    ocp.dims.N = predict_node_num
    ocp.model = create_diffdrive_model()

    # cost
    ocp.parameter_values = np.zeros(5)

    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.W = mat_W
    ocp.cost.yref = np.zeros(7)

    ocp.cost.cost_type_e = "NONLINEAR_LS"
    ocp.cost.W_e = mat_We
    ocp.cost.yref_e = np.zeros(5)

    # constraints
    ocp.constraints.lh = np.array([0, -max_wheel_linear_vel, -max_wheel_linear_vel])
    ocp.constraints.uh = np.ones(3) * max_wheel_linear_vel
    ocp.constraints.lh_e = ocp.constraints.lh
    ocp.constraints.uh_e = ocp.constraints.uh
    ocp.constraints.x0 = np.zeros(7)
    ocp.constraints.idxbu = np.array([0, 1])
    ocp.constraints.lbu = np.array([-max_linear_acc, -max_angular_acc])
    ocp.constraints.ubu = -ocp.constraints.lbu

    # configure solver
    ocp.solver_options.tf = predict_time_horizon
    ocp.solver_options.qp_solver_iter_max = 10
    ocp.solver_options.levenberg_marquardt = 1e-3
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"

    # export settings
    ocp.acados_include_path = acados_include_path
    ocp.acados_lib_path = acados_lib_path
    ocp.code_export_directory = code_export_dir

    return ocp


# also need to define simulator for a different model update time interval
def define_sim() -> AcadosSim:
    sim = AcadosSim()
    sim.model = create_diffdrive_model()

    sim.solver_options.T = sim_dt
    sim.parameter_values = np.zeros(5)

    sim.acados_include_path = acados_include_path
    sim.acados_lib_path = acados_lib_path
    sim.code_export_directory = code_export_dir

    return sim


# generate code
if __name__ == "__main__":
    problem = define_ocp()
    simulator = define_sim()

    if os.path.exists(code_export_dir):
        shutil.rmtree(code_export_dir)

    json_ocp = (code_export_dir +
                "/acados_ocp_solver_config_" + problem.model.name + ".json")
    ocp_solver = AcadosOcpSolver(problem, json_ocp)

    json_sim = (code_export_dir +
                "/acados_sim_solver_config_" + problem.model.name + ".json")
    sim_solver = AcadosSimSolver(simulator, json_sim)
