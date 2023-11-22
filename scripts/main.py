import numpy as np
import time
import matplotlib.pyplot as plt
from fancy_plots import fancy_plots_3, fancy_plots_4
from fancy_plots import plot_states_position, plot_states_quaternion, plot_control_actions, plot_states_euler
from system_functions import f_d, axisToquaternion, f_d_casadi, ref_trajectory, compute_desired_quaternion, get_euler_angles
from export_ode_model import quadrotorModel
from acados_template import AcadosOcpSolver, AcadosSimSolver
from nmpc import create_ocp_solver



def main(ts: float, t_f: float, t_N: float, x_0: np.ndarray, L: list)-> None:
    # DESCRIPTION
    # simulation of a quadrotor using a NMPC as a controller
    # INPUTS
    # ts                        - sample time
    # t_f                       - final time
    # t_N                       - NMPC prediction time
    # x_0                       - initial conditions
    # OUTPUS 
    # x                         - states of the system
    # F                         - force control action
    # M                         - torques of the system
    # delta_t                   - computational time

    # Simulation time definition
    t = np.arange(0, t_f + ts, ts)

    # Prediction Node of the NMPC formulation
    N = np.arange(0, t_N + ts, ts)
    N_prediction = N.shape[0]

    # Auxiliary variables for the execution time of the NMPC
    delta_t = np.zeros((1, t.shape[0] - N_prediction), dtype=np.double)
    t_sample = ts*np.ones((1, t.shape[0] - N_prediction), dtype=np.double)

    # Generalized states of the system
    x = np.zeros((13, t.shape[0] + 1 - N_prediction), dtype=np.double)
    x[:, 0] = x_0

    # Euler angles of the system
    euler = np.zeros((3, t.shape[0] + 1 - N_prediction), dtype=np.double)
    euler[:, 0] = get_euler_angles(x[6:10, 0])

    # Control actions
    F = np.zeros((1, t.shape[0] - N_prediction), dtype=np.double)
    M = np.zeros((3, t.shape[0] - N_prediction), dtype=np.double)

    # Generalized control actions
    u = np.zeros((4, t.shape[0] - N_prediction), dtype=np.double)

    # Desired states
    xref = np.zeros((13, t.shape[0]), dtype = np.double)
    # Desired position of the system
    xd, yd, theta, theta_p = ref_trajectory(t)

    xref[0, :] = xd
    xref[1, :] = yd
    xref[2, :] = 0.0

    # Compute desired quaternion
    q_d = compute_desired_quaternion(theta, theta_p, t, ts)

    # Desired quaternion
    xref[6, :] = q_d[0, :]
    xref[7, :] = q_d[1, :]
    xref[8, :] = q_d[2, :]
    xref[9, :] = q_d[3, :]

    # Desired Euler angles
    euler_d = np.zeros((3, t.shape[0]), dtype=np.double)
    euler_d[2, :] = theta

    # Constraints on control actions
    F_max = L[0]*L[4] + 10
    F_min = 0
    tau_1_max = 0.1
    tau_1_min = -0.1
    tau_2_max = 0.1
    tau_2_min = -0.1
    tau_3_max = 0.1
    taux_3_min = -0.1

    # Quadrotor Model based on casadi
    model, f_d_c, constraint, f_error = quadrotorModel(L)

    # Optimization problem
    ocp = create_ocp_solver(x[:, 0], N_prediction, t_N, F_max, F_min, tau_1_max, tau_1_min, tau_2_max, tau_2_min, tau_3_max, taux_3_min, L)

    # Creation of the optimization problem
    solver_json = 'acados_ocp_' + model.name + '.json'

    # These parameters can be modified in order to avoid the creation of a new optimal control problem 
    AcadosOcpSolver.generate(ocp, json_file=solver_json)
    AcadosOcpSolver.build(ocp.code_export_directory, with_cython=True)

    # Assignation of the optimal control problem
    acados_ocp_solver = AcadosOcpSolver.create_cython_solver(solver_json)
    
    # Integration using Acados
    acados_integrator = AcadosSimSolver(ocp, json_file="acados_ocp_" + ocp.model.name + ".json")

    # Auxiliary variables and control
    nx = ocp.model.x.size()[0]
    nu = ocp.model.u.size()[0]

    # Initial States Acados
    for stage in range(N_prediction + 1):
        acados_ocp_solver.set(stage, "x", x[:, 0])
    for stage in range(N_prediction):
        acados_ocp_solver.set(stage, "u", np.zeros((nu,)))


    # Loop simulation
    for k in range(0, t.shape[0] - N_prediction):
        tic = time.time()
        # Control Law Acados
        acados_ocp_solver.set(0, "lbx", x[:, k])
        acados_ocp_solver.set(0, "ubx", x[:, k])

        # Desired Trajectory of the system
        for j in range(N_prediction):
            yref = xref[:,k+j]
            acados_ocp_solver.set(j, "p", yref)

        yref_N = xref[:,k+N_prediction]
        acados_ocp_solver.set(N_prediction, "p", yref_N)

        # Check Solution since there can be possible errors 
        status = acados_ocp_solver.solve()

        # Get the control Action
        aux_control = acados_ocp_solver.get(0, "u")
        F[:, k] = aux_control[0]
        M[:, k] = aux_control[1:4]
        u[0, k] = F[:, k]
        u[1:4, k] = M[:, k]

        # run time
        toc_solver = time.time() - tic
        delta_t[:, k] = toc_solver

        # Get states of the system using acados (ERK)
        acados_integrator.set("x", x[:, k])
        acados_integrator.set("u", u[:, k])

        status_integral = acados_integrator.solve()
        xcurrent = acados_integrator.get("x")

        # System evolution
        x[:, k+1] = xcurrent
        euler[:, k+1] = get_euler_angles(x[6:10, k+1])

    # Results
    # Position
    fig11, ax11, ax21, ax31 = fancy_plots_3()
    plot_states_position(fig11, ax11, ax21, ax31, x[0:3, :], xref[0:3, :], t, "Position of the System")
    plt.show()

    # Orientation
    fig12, ax12, ax22, ax32, ax42 = fancy_plots_4()
    plot_states_quaternion(fig12, ax12, ax22, ax32, ax42, x[6:10, :], xref[6:10, :], t, "Quaternions of the System")
    plt.show()

    # Control Actions
    fig13, ax13, ax23, ax33, ax43 = fancy_plots_4()
    plot_control_actions(fig13, ax13, ax23, ax33, ax43, F, M, t, "Control Actions of the System")
    plt.show()

    fig14, ax14, ax24, ax34 = fancy_plots_3()
    plot_states_euler(fig14, ax14, ax24, ax34, euler[0:3, :], euler_d[0:3, :], t, "Euler Angles of the System")
    plt.show()
    None

if __name__ == '__main__':
    try: #################################### Simulation  #####################################################
        # Time parameters
        ts = 0.05
        t_f = 30
        t_N = 0.5

        # Parameters of the system  (mass, inertial matrix, gravity)
        m = 1                                                                             
        Jxx = 2.64e-3
        Jyy = 2.64e-3
        Jzz = 4.96e-3
        g = 9.8
        L = [m, Jxx, Jyy, Jzz, g]

        # Initial conditions of the system
        pos_0 = np.array([2.0, 2.0, 1.0], dtype=np.double)
        vel_0 = np.array([0.0, 0.0, 0.0], dtype=np.double)
        angle_0 = 3.81
        axis_0 = [0.4896, 0.2032, 0.8480]
        quat_0 = axisToquaternion(angle_0, axis_0)
        omega_0 = np.array([0.0, 0.0, 0.0], dtype=np.double)

        # Generalized vector of initial conditions
        x_0 = np.hstack((pos_0, vel_0, quat_0, omega_0))

        # Simulation
        main(ts, t_f, t_N, x_0, L)

    except(KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass