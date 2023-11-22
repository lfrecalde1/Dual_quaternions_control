from acados_template import AcadosModel
from casadi import Function, MX, vertcat, sin, cos
import numpy as np
import casadi as ca

def quatTorot_c(quat):
    # Function to transform a quaternion to a rotational matrix
    # INPUT
    # quat                                                       - unit quaternion
    # OUTPUT                                     
    # R                                                          - rotational matrix

    # Normalized quaternion
    q = quat
    q = q/ca.norm_2(q)

    # Create empty variable
    q_hat = ca.MX.zeros(3, 3)
    q_hat[0, 1] = -q[3]
    q_hat[0, 2] = q[2]
    q_hat[1, 2] = -q[1]
    q_hat[1, 0] = q[3]
    q_hat[2, 0] = -q[2]
    q_hat[2, 1] = q[1]

    # Compute Rotational Matrix
    R = ca.MX.eye(3) + 2 * (q_hat@q_hat) + 2 * q[0] * q_hat
    return R

def quatdot_c(quat, omega):
    # Quaternion evolution guaranteeing norm 1 (Improve this section)
    # INPUT
    # quat                                                   - actual quaternion
    # omega                                                  - angular velocities
    # OUTPUT
    # qdot                                                   - rate of change of the quaternion
    # Split values quaternion
    qw = quat[0, 0]
    qx = quat[1, 0]
    qy = quat[2, 0]
    qz = quat[3, 0]

    # Split angular values
    p = omega[0, 0]
    q = omega[1, 0]
    r = omega[2, 0]

    # Auxiliary variable in order to avoid numerical issues
    K_quat = 2
    quat_error = 1 - (qw**2 + qx**2 + qy**2 + qz**2)

    # Create skew matrix
    S = ca.vertcat(
        ca.horzcat(0.0, -p, -q, -r),
        ca.horzcat(p, 0.0, r, -q),
        ca.horzcat(q, -r, 0.0, p),
        ca.horzcat(r, q, -p, 0.0))

    q_dot = (1/2)*(S@quat) + K_quat*quat_error*quat
    return q_dot

def quadrotorModel(L: list)-> AcadosModel:
    # Dynamics of the quadrotor based on unit quaternions
    # INPUT
    # L                                                          - system parameters(mass, Inertias and gravity)
    # OUTPUT                           
    # model                                                      - Acados model
    model_name = 'quadrotor'
    # Split system parameters
    m = L[0]
    Jxx = L[1]
    Jyy = L[2]
    Jzz = L[3]
    gravity = L[4]

    # States of the system
    x = MX.sym('x')
    y = MX.sym('y')
    z = MX.sym('z')

    vx = MX.sym('vx')
    vy = MX.sym('vy')
    vz = MX.sym('vz')

    qw = MX.sym('qw')
    q1 = MX.sym('q1')
    q2 = MX.sym('q2')
    q3 = MX.sym('q3')

    wx = MX.sym('wx')
    wy = MX.sym('wy')
    wz = MX.sym('wz')

    X = vertcat(x, y, z, vx, vy, vz, qw, q1, q2, q3, wx, wy, wz)

    # Auxiliary variables implicit function
    x_dot = MX.sym('x_dot')
    y_dot = MX.sym('y_dot')
    z_dot = MX.sym('z_dot')

    vx_dot = MX.sym('vx_dot')
    vy_dot = MX.sym('vy_dot')
    vz_dot = MX.sym('vz_dot')

    qw_dot = MX.sym('qw_dot')
    q1_dot = MX.sym('q1_dot')
    q2_dot = MX.sym('q2_dot')
    q3_dot = MX.sym('q3_dot')

    wx_dot = MX.sym('wx_dot')
    wy_dot = MX.sym('wy_dot')
    wz_dot = MX.sym('wz_dot')

    X_dot = vertcat(x_dot, y_dot, z_dot, vx_dot, vy_dot, vz_dot, qw_dot, q1_dot, q2_dot, q3_dot, wx_dot, wy_dot, wz_dot)

    # Control actions
    F_ref = MX.sym('F_ref')
    tau_1_ref = MX.sym('tau_1_ref')
    tau_2_ref = MX.sym('tau_2_ref')
    tau_3_ref = MX.sym('tau_3_ref')

    u = vertcat(F_ref, tau_1_ref, tau_2_ref, tau_3_ref)
    M = vertcat(tau_1_ref, tau_2_ref, tau_3_ref)
    F = vertcat(F_ref)

    # Inertial Matrix
    J = MX.zeros(3, 3)
    J[0, 0] = Jxx
    J[1, 1] = Jyy
    J[2, 2] = Jzz

    #Auxiliar variable
    e3 = MX.zeros(3, 1)
    e3[2, 0] = 1.0
    g = gravity*e3

    # Split values
    vel = X[3:6, 0]
    quat = X[6:10, 0]
    omega = X[10:13, 0]
    R = quatTorot_c(quat)

    # Rate of change of the system
    acc = ((u[0]*(R@e3))/m) - g

    qdot = quatdot_c(quat, omega)
    aux = J@omega
    aux_cross = ca.cross(omega, aux)
    omega_dot = ca.inv(J)@(u[1:4] - aux_cross)

    # Desired Trajectory
    x_d = MX.sym('x_d')
    y_d = MX.sym('y_d')
    z_d = MX.sym('z_d')

    vx_d = MX.sym('vx_d')
    vy_d = MX.sym('vy_d')
    vz_d = MX.sym('vz_d')

    qw_d = MX.sym('qw_d')
    q1_d = MX.sym('q1_d')
    q2_d = MX.sym('q2_d')
    q3_d = MX.sym('q3_d')

    wx_d = MX.sym('wx_d')
    wy_d = MX.sym('wy_d')
    wz_d = MX.sym('wz_d')

    X_d = vertcat(x_d, y_d, z_d, vx_d, vy_d, vz_d, qw_d, q1_d, q2_d, q3_d, wx_d, wy_d, wz_d)
    p = X_d

    # Explicit and implicit functions
    f_expl = vertcat(vel, acc, qdot, omega_dot)
    f_system = Function('system',[X, u], [f_expl])
    f_impl = X_dot - f_expl

    # Algebraic variables
    z = []

    # Dynamics
    model = AcadosModel()
    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = X
    model.xdot = X_dot
    model.u = u
    model.z = z
    model.p = p
    model.name = model_name
    return model, f_system

def conjugate_quaternion(q):
    # Compute the conjugate of a specified quaternion
    # INPUT
    # q                                              - quaternion
    # OUTPUT 
    # q_c                                            - conjugate quaternion
    a1 = q[0]
    b1 = -q[1]
    c1 = -q[2]
    d1 = -q[3]
    q_c = vertcat(a1, b1, c1, d1)
    return q_c
def matrix_q(q):
    # Compute Q matrix for quaternion multiplication
    #INPUT
    # q                                                - quaternion
    #OUTPUT  
    # Q                                                - quaternion Matrix
    # Split variables
    a1 = q[0]
    b1 = q[1]
    c1 = q[2]
    d1 = q[3]

    Q = ca.vertcat(
        ca.horzcat(a1, -b1, -c1, -d1),
        ca.horzcat(b1, a1, -d1, c1),
        ca.horzcat(c1, d1, a1, -b1),
        ca.horzcat(d1, -c1, b1, a1))
    return Q

def quat_multiply(q1, q2):
    # Multiplication between quaternions
    # INPUT
    # q1                              - quaternion 1
    # q2                              - quaternion 2
    #OUTPUT
    # q1q2                            - q1 multiply q2
    Q = matrix_q(q1)
    q1q2 = Q@q2
    return q1q2



