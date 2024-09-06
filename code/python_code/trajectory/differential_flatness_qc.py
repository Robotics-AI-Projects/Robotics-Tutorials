# """
# Refer to :
# [Minimum Snap Trajectory Generation and Control for Quadrotors](https://web.archive.org/web/20120713162030id_/http://www.seas.upenn.edu/~dmel/mellingerICRA11.pdf)

# TODO:
# Fixed second derivatives equations of motion of quadcopter for calculating control reference
# """

# import numpy as np

# def trajectory_ref(t_ref, config, trajectory):
#     I = np.diag(np.array([config["Ixx"], config["Iyy"], config["Izz"]]))

#     def numerical_derivative(func, t, dt=t_ref[4]-t_ref[3]):
#         return (func(t + dt) - func(t - dt)) / (2 * dt)

#     pos = trajectory
#     vel = lambda t: numerical_derivative(pos, t)
#     acc = lambda t: numerical_derivative(vel, t)
#     jerk = lambda t: numerical_derivative(acc, t)
#     snap = lambda t: numerical_derivative(jerk, t)

#     u_control = []
#     state_ref = []
#     traj_ref = []

#     for t in t_ref:
#         x, y, z, psi = pos(t)
#         traj_ref.append([x, y, z])
#         dx, dy, dz, dpsi = vel(t)
#         d2x, d2y, d2z, d2psi = acc(t)
#         d3x, d3y, d3z, d3psi = jerk(t)
#         d4x, d4y, d4z, d4psi = snap(t)

#         ag = np.array([d2x, d2y, d2z + config["g"]])
#         z_B = ag / np.linalg.norm(ag)

#         x_c = np.array([np.cos(psi), np.sin(psi), 0])
#         y_c = np.array([-np.sin(psi), np.cos(psi), 0])

#         x_B = np.cross(y_c, z_B)/np.linalg.norm(np.cross(y_c, z_B))
#         y_B = np.cross(z_B, x_B)
#         y_B = y_B / np.linalg.norm(y_B)

#         R = np.array([x_B, y_B, z_B]).T

#         phi = np.arctan2(R[2, 1], R[2, 2])
#         theta = np.arctan2(-R[2, 0], np.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2))
#         psi = np.arctan2(R[1, 0], R[0, 0])

#         da = np.array([d3x, d3y, d3z])
#         T = config["m"] * np.linalg.norm(ag)

#         z_W = np.array([0., 0., 1.])
#         h_omega = config["m"] / T * (da - (z_B.dot(da)) * z_B)
#         p = -config["m"] * np.dot(y_B.T, da)/ T
#         q = config["m"] * np.dot(x_B.T, da)/ T
#         r = dpsi * np.dot(x_c.T, x_B) + q * np.dot(y_c.T, z_B) / np.linalg.norm(np.cross(y_c,z_B))

#         zeta = np.array([x, y, z])
#         eta = np.array([phi, theta, psi])
#         nu = np.array([dx, dy, dz])
#         omega = np.array([p, q, r])

#         state = np.hstack([zeta, eta, nu, omega])
#         state_ref.append(state)

#         ### Control Ref
#         d2a = np.array([d4x, d4y,d4z])
#         dT = np.dot(z_B.T, config["m"]*da)

#         dp = (np.dot(x_B.T, d2a) - 2 * dT * q - T * p *r ) * config["m"]/ T 
#         dq = -(np.dot(y_B.T, d2a) - 2 * dT * p - T * q *r ) * config["m"]/ T
#         dr = (dq * y_c.T.dot(z_B) 
#               + d2psi * x_c.T.dot(x_B)
#               - 2.0 * dpsi * q * x_c.T.dot(z_B)
#               + 2.0 * dpsi * r * x_c.T.dot(y_B)
#               - p * q * y_c.T.dot(y_B)
#               - p * r * y_c.T.dot(z_B)
#             )/ np.linalg.norm(np.cross(y_c, z_B))
        
#         omega_dot = np.array([dp, dq, dr])

#         tau_phi, tau_theta, tau_psi= I @ omega_dot + np.cross(omega, I @ omega)

#         torque = np.hstack([T, tau_phi, tau_theta, tau_psi])
#         tor_mat = np.array(
#             [[config["k"], config["k"], config["k"], config["k"]],
#              [-config["k"] * config["l"], 0.0, config["k"] * config["l"], 0.0],
#              [0.0, -config["k"] * config["l"], 0.0, config["k"] * config["l"]],
#              [-config["b"], config["b"], -config["b"], config["b"]],])
#         u_motor = np.linalg.inv(tor_mat) @ torque
#         u_control.append(u_motor)

#     return np.array(u_control), np.array(state_ref), np.array(traj_ref)

"""
Copied from :
PyCollimator - Differential Flatness
"""

import numpy as np


def T_angular_inv(phi, theta, psi):
    """
    Transformation matrix of angular velocity from body frame to inertial frame.
    Input : phi, theta in radians
    Output : inverse transformation matrix
    """
    T_matrix = np.array([[1, np.sin(phi) * np.tan(theta), np.cos(phi) * np.tan(theta)],
                         [0, np.cos(phi), -np.sin(phi)],
                         [0, np.sin(phi) / np.cos(theta), np.cos(phi) / np.cos(theta)]])
    return T_matrix

def trajectory_ref(t_ref, config, trajectory):
    I = np.diag(np.array([config["Ixx"], config["Iyy"], config["Izz"]]))

    def numerical_derivative(func, t, dt=t_ref[4]-t_ref[3]):
        return (func(t + dt) - func(t - dt)) / (2 * dt)

    pos = trajectory
    vel = lambda t: numerical_derivative(pos, t)
    acc = lambda t: numerical_derivative(vel, t)
    jerk = lambda t: numerical_derivative(acc, t)
    snap = lambda t: numerical_derivative(jerk, t)

    u_control = []
    state_ref = []
    traj_ref = []

    for t in t_ref:
        x, y, z, psi = pos(t)
        traj_ref.append([x, y, z])
        dx, dy, dz, dpsi = vel(t)
        d2x, d2y, d2z, d2psi = acc(t)
        d3x, d3y, d3z, d3psi = jerk(t)
        d4x, d4y, d4z, d4psi = snap(t)

        alpha = np.array([d2x, d2y, d2z + config["g"]])
        beta = alpha

        xc = np.array([np.cos(psi), np.sin(psi), 0])
        yc = np.array([-np.sin(psi), np.cos(psi), 0])

        xb = np.cross(yc, alpha) / np.linalg.norm(np.cross(yc, alpha))
        yb = np.cross(beta, xb) / np.linalg.norm(np.cross(beta, xb))
        zb = np.cross(xb, yb)

        R = np.vstack([xb, yb, zb]).T
        phi = np.arctan2(R[2, 1], R[2, 2])
        theta = np.arctan2(-R[2, 0], np.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2))
        psi = np.arctan2(R[1, 0], R[0, 0])  # psi

        eta = np.array([phi, theta, psi])

        # Compute angular velocities
        c = zb.dot(alpha)
        dot_a = np.array([d3x, d3y, d3z])

        q = xb.dot(dot_a) / c
        p = -yb.dot(dot_a) / c
        r = (dpsi * xc.dot(xb) + q * yc.dot(zb)) / np.linalg.norm(np.cross(yc, zb))

        nu = np.array([p, q, r])

        Wn_inv = T_angular_inv(phi, theta, psi)

        dot_eta = Wn_inv @ nu

        xi = np.array([x, y, z])
        dot_xi = np.array([dx, dy, dz])
        state = np.hstack([xi, eta, dot_xi, dot_eta])
        state_ref.append(state)

        # Compute reference control
        dot_c = zb.dot(dot_a)
        ddot_a = np.array([d4x, d4y, d4z])

        dot_q = (1.0 / c) * (xb.dot(ddot_a) - 2.0 * dot_c * q - c * p * r)
        dot_p = (1.0 / c) * (-yb.dot(ddot_a) - 2.0 * dot_c * p + c * q * r)
        dot_r = (1.0 / np.linalg.norm(np.cross(yc, zb))) * (
            dot_q * yc.dot(zb)
            + d2psi * xc.dot(xb)
            + 2.0 * dpsi * r * xc.dot(yb)
            - 2.0 * dpsi * q * xc.dot(zb)
            - p * q * yc.dot(yb)
            - p * r * yc.dot(zb)
        )

        dot_nu = np.array([dot_p, dot_q, dot_r])

        tau_b = I @ dot_nu + np.cross(nu, I @ nu)
        torque = np.hstack([config["m"]*c, tau_b])
        tor_mat = np.array(
            [[config["k"], config["k"], config["k"], config["k"]],
             [0.0, -config["k"] * config["l"], 0.0, config["k"] * config["l"]],
             [-config["k"] * config["l"], 0.0, config["k"] * config["l"], 0.0],
             [-config["b"], config["b"], -config["b"], config["b"]],])
        u_motor = np.linalg.inv(tor_mat) @ torque
        u_control.append(u_motor)

    return np.array(u_control), np.array(state_ref), np.array(traj_ref)