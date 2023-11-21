import numpy as np
import time
import matplotlib.pyplot as plt
from fancy_plots import fancy_plots_3, fancy_plots_4
from fancy_plots import plot_states_position, plot_states_quaternion, plot_control_actions
from system_functions import f_d, axisToquaternion
def main(ts: float, t_f: float, t_N: float, x_0: np.ndarray, L: list)-> None:
    # DESCRIPTION
    # simulation of a quadrotor using a NMPC as a controller
    # INPUTS
    # ts                        - sample time
    # t_f                       - final time
    # t_N                       - NMPC prediction time
    # x_0                       - initial conditions
    # OUTPUS 
    # None

    # Simulation time definition
    t = np.arange(0, t_f + ts, ts)

    # Prediction Node of the NMPC formulation
    N = np.arange(0, t_N + ts, ts)
    N_prediction = N.shape[0]

    # Auxiliary variables for the execution time of the NMPC and the sample time
    delta_t = np.zeros((1, t.shape[0] - N_prediction), dtype=np.double)
    t_sample = ts*np.ones((1, t.shape[0] - N_prediction), dtype=np.double)

    # Vector of the system
    x = np.zeros((13, t.shape[0] + 1 - N_prediction), dtype=np.double)
    x[:, 0] = x_0

    # Control actions
    F = np.zeros((1, t.shape[0] - N_prediction), dtype=np.double)
    M = np.zeros((3, t.shape[0] - N_prediction), dtype=np.double)

    # Loop simulation
    for k in range(0, t.shape[0] - N_prediction):
        tic = time.time()
        F[:, k] = L[0]*L[4]

        # run time
        toc_solver = time.time() - tic
        delta_t[:, k] = toc_solver

        # System evolution
        x[:, k+1] = f_d(x[:, k], F[:, k], M[:, k], ts, L)
    # Results
    # Position
    fig11, ax11, ax21, ax31 = fancy_plots_3()
    plot_states_position(fig11, ax11, ax21, ax31, x[0:3, :], t, "Position of the System")
    plt.show()

    # Orientation
    fig12, ax12, ax22, ax32, ax42 = fancy_plots_4()
    plot_states_quaternion(fig12, ax12, ax22, ax32, ax42, x[6:10, :], t, "Quaternions of the System")
    plt.show()

    # Control Actions
    fig13, ax13, ax23, ax33, ax43 = fancy_plots_4()
    plot_control_actions(fig13, ax13, ax23, ax33, ax43, F, M, t, "Control Actions of the System")
    plt.show()

    None

if __name__ == '__main__':
    try:
        # Time parameters
        ts = 0.05
        t_f = 30
        t_N = 0.5

        # Parameters of the system
        m = 1
        Jxx = 2.64e-3
        Jyy = 2.64e-3
        Jzz = 4.96e-3
        g = 9.8
        L = [m, Jxx, Jyy, Jzz, g]

        # Initial conditions
        pos_0 = np.array([0.0, 0.0, 0.0], dtype=np.double)
        vel_0 = np.array([0.0, 0.0, 0.0], dtype=np.double)
        angle_0 = 0.0
        axis_0 = [0.0, 0.0, 1.0]
        quat_0 = axisToquaternion(angle_0, axis_0)
        omega_0 = np.array([0.0, 0.0, 0.0], dtype=np.double)

        # Vector of initial conditions
        x_0 = np.hstack((pos_0, vel_0, quat_0, omega_0))

        # Simulation
        main(ts, t_f, t_N, x_0, L)

    except(KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass