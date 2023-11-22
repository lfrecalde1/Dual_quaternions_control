from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from export_ode_model import quadrotorModel
from casadi import Function, MX, vertcat, sin, cos
import numpy as np

def create_ocp_solver(x0, N_horizon, t_horizon, F_max, F_min, tau_1_max, tau_1_min, tau_2_max, tau_2_min, tau_3_max, tau_3_min, L)->AcadosOcp:
    # Creation of the optimal control problem

    # Optimization class
    ocp = AcadosOcp()

    # Modelof the sytem
    model, f_d = quadrotorModel(L)

    # Variables assignation
    ocp.model = model
    ocp.p = model.p
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu

    # Set Dimensions
    ocp.dims.N = N_horizon

    # Gain matrices
    Q = MX.zeros(3, 3)
    Q[0, 0] = 1.0
    Q[1, 1] = 1.0
    Q[2, 2] = 1.0

    R = MX.zeros(4, 4)
    R[0, 0] = 1/F_max
    R[1, 1] = 10/tau_1_max
    R[2, 2] = 10/tau_2_max
    R[3, 3] = 10/tau_3_max

    # Definition of the cost functions
    ocp.cost.cost_type = "EXTERNAL"
    ocp.cost.cost_type_e = "EXTERNAL"


    error_position = ocp.p[0:3] - model.x[0:3]
    ocp.model.cost_expr_ext_cost = 1*(error_position.T @ Q @error_position) + 1.5*(model.u[0:4].T @ R @ model.u[0:4])
    ocp.model.cost_expr_ext_cost_e = 1*(error_position.T @ Q @error_position)
    
    ocp.parameter_values = np.zeros(nx)

    # Set constraints
    ocp.constraints.lbu = np.array([F_min, tau_1_min, tau_2_min, tau_3_min])
    ocp.constraints.ubu = np.array([F_max, tau_1_max, tau_2_max, tau_3_max])
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])
    ocp.constraints.x0 = x0

    # Set options
    ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"  # FULL_CONDENSING_QPOASES
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"  # 'GAUSS_NEWTON', 'EXACT'
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"  # SQP_RTI, SQP

    # Set prediction horizon
    ocp.solver_options.tf = t_horizon


    return ocp