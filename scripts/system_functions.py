import numpy as np
def quatTorot(quat: np.ndarray)->np.ndarray:
    # Function to transform a quaternion to a rotational matrix
    # INPUT
    # quat                                                       - unit quaternion
    # OUTPUT                                     
    # R                                                          - rotational matrix
    # Normalized quaternion
    q = quat
    q = q/np.linalg.norm(q, 2)

    # Auxiliar variable
    q_hat = np.zeros((3, 3), dtype=np.double)
    q_hat[0, 1] = -q[3]
    q_hat[0, 2] = q[2]
    q_hat[1, 2] = -q[1]
    q_hat[1, 0] = q[3]
    q_hat[2, 0] = -q[2]
    q_hat[2, 1] = q[1]

    R = np.eye(3) + 2 * (q_hat@q_hat) + 2 * q[0] * q_hat
    return R
def quatdot(quat: np.ndarray, omega: np.ndarray)-> np.ndarray:
    # Quaternion evolution guaranteeing norm 1.
    # INPUT
    # quat                                                   - actual quaternion
    # omega                                                  - angular velocities
    # OUTPUT
    # qdot                                                   - rate of change of the quaternion
    # Split values quaternion
    quat = quat.reshape((4, 1))
    qw = quat[0, 0]
    qx = quat[1, 0]
    qy = quat[2, 0]
    qz = quat[3, 0]

    # Split angular values
    p = omega[0]
    q = omega[1]
    r = omega[2]

    # Auxiliary variable in order to avoid numerical issues
    K_quat = 2
    quat_error = 1 - (qw**2 + qx**2 + qy**2 + qz**2)

    # Create skew matrix
    S = np.array([[0.0, -p, -q, -r], [p, 0.0, r, -q], [q, -r, 0.0, p], [r, q, -p, 0.0]], dtype=np.double)
    q_dot = (1/2)*(S@quat) + K_quat*quat_error*quat
    return q_dot

def systemdynamics(x: np.ndarray, F: np.ndarray, M: np.ndarray, L: list)-> np.ndarray:
    # Dynamics of the quadrotor based on unit quaternions
    # INPUT
    # x                                                          - states of the system (13, )
    # F                                                          - Force (1, )
    # M                                                          - Torques(3, )
    # L                                                          - system parameters(mass, Inertias and gravity)
    # OUTPUT                           
    # xdot                                                       - The rate of change of flow in the system. (13, )

    # Split system parameters
    m = L[0]
    Jxx = L[1]
    Jyy = L[2]
    Jzz = L[3]
    gravity = L[4]

    # Inertial Matrix
    J = np.array([[Jxx, 0.0, 0.0], [0.0, Jyy, 0.0], [0.0, 0.0, Jzz]], dtype=np.double)

    # Axiliar variable
    e3 = np.array([[0.0], [0.0], [1.0]], dtype=np.double)
    g = gravity*e3

    # Split states values
    vel = x[3:6]
    quat = x[6:10]
    omega = x[10:13]
    R = quatTorot(quat)

    # Control actions
    M = M.reshape((3, 1))
    F = F

    # Rate of change
    # Linear section
    vel = vel.reshape((3, 1))
    acc = ((F*(R@e3))/m) - g

    # Angular
    qdot = quatdot(quat, omega)
    aux = J@omega
    aux_cross = np.cross(omega, aux)
    aux_cross = aux_cross.reshape((3, 1))
    omega_dot = np.linalg.inv(J)@(M - aux_cross)
    
    # Stack Vectors
    xdot = np.vstack((vel, acc, qdot, omega_dot))
    return xdot[:, 0]

    
def axisToquaternion(angle: float, axis: list)-> np.ndarray:
    # DESCRIPTION
    # transform axis-angle to quaternion
    # INPUTS
    # anlge                                       - angle of rotation
    # axis                                        - axis where the rotation is executed
    # OUTPUT
    # quat                                        - unit quaternion
    axis_aux = np.array(axis)

    real_part = np.cos((angle)/2)
    imaginary_part = axis_aux*np.sin((angle)/2)

    quat = [real_part, imaginary_part[0], imaginary_part[1], imaginary_part[2]]
    quat = np.array(quat)
    return quat

def f_d(x: np.ndarray, F: np.ndarray, M: np.ndarray, ts: float, L: list):
    # Compute Runge Kutta
    # INPUT
    # x_k                                                          - states of the system (13, )
    # F_k                                                          - Force (1, )
    # M_k                                                          - Torques(3, )
    # L                                                            - system parameters(mass, Inertias and gravity)
    # ts                                                           - sample time
    # OUTPUT                           
    # x_k+1                                                        - The rate of change of flow in the system. (13, )

    k1 = systemdynamics(x, F, M, L)
    k2 = systemdynamics(x+(ts/2)*k1, F, M, L)
    k3 = systemdynamics(x+(ts/2)*k2, F, M, L)
    k4 = systemdynamics(x+(ts)*k3, F, M, L)
    x_k = x + (ts/6)*(k1 +2*k2 +2*k3 +k4)
    return x_k

def f_d_casadi(x: np.ndarray, F: np.ndarray, M: np.ndarray, ts: float, f_s):
    # Compute Runge Kutta
    # INPUT
    # x_k                                                          - states of the system (13, )
    # F_k                                                          - Force (1, )
    # M_k                                                          - Torques(3, )
    # ts                                                           - sample time
    # f_s                                                          - casadi dynamics
    # OUTPUT                           
    # x_k+1                                                        - The rate of change of flow in the system. (13, )

    k1 = f_s(x, F, M)
    k2 = f_s(x+(ts/2)*k1, F, M)
    k3 = f_s(x+(ts/2)*k2, F, M)
    k4 = f_s(x+(ts)*k3, F, M)
    x_k = x + (ts/6)*(k1 +2*k2 +2*k3 +k4)
    aux_x = np.array(x_k[:, 0]).reshape(13, )
    return aux_x