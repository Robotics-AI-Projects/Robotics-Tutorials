"""
Refer to :
[Minimum Snap Trajectory Generation and Control for Quadrotors](https://web.archive.org/web/20120713162030id_/http://www.seas.upenn.edu/~dmel/mellingerICRA11.pdf)

TODO:
Fixed second derivatives equations of motion of quadcopter for calculating control reference
"""

import numpy as np

def trajectory_ref(t_ref, config, trajectory):
    I = np.diag(np.array([config.Ixx, config.Iyy, config.Izz]))

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

        ag = np.array([d2x, d2y, d2z + config.g])
        z_B = ag / np.linalg.norm(ag)

        x_c = np.array([np.cos(psi), np.sin(psi), 0])
        y_B = np.cross(z_B, x_c)
        y_B = y_B / np.linalg.norm(y_B)
        x_B = np.cross(y_B, z_B)
        R = np.array([x_B, y_B, z_B]).T

        phi = np.arctan2(R[2, 1], R[2, 2])
        theta = np.arctan2(-R[2, 0], np.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2))
        psi = np.arctan2(R[1, 0], R[0, 0])

        da = np.array([d3x, d3y, d3z])
        T = config.m * np.linalg.norm(ag)

        z_W = np.array([0., 0., 1.])
        h_omega = config.m / T * (da - (z_B.dot(da)) * z_B)
        p = -np.dot(h_omega, y_B)
        q = np.dot(h_omega, x_B)
        r = dpsi * np.dot(z_W, z_B)

        zeta = np.array([x, y, z])
        eta = np.array([phi, theta, psi])
        nu = np.array([dx, dy, dz])
        omega = np.hstack([p, q, r])

        state = np.concatenate([zeta, eta, nu, omega])
        state_ref.append(state)

        ### Control Ref
        d2a = np.array([d4x, d4y,d4z])
        dT = np.dot(z_B, config.m*da)
        d2T = config.m * (np.dot(dz, da) + np.dot(z_B, d2a))
        dh_omega = ((config.m * d2a -  d2T*z_B - 2 * (dT * h_omega))/T) - np.cross(omega, h_omega)

        dx_c = np.array([-np.sin(psi)*d2psi, np.cos(psi)*d2psi, 0])

        A = np.cross(z_B, x_c)
        dA = np.cross(h_omega, x_c) + np.cross(z_B, dx_c)

        dy = dA/np.linalg.norm(A) - A * ((A.T * dA) / np.linalg.norm(A)**3)
        dx = np.cross(dy, z_B) + np.cross(y_B, h_omega)

        dp = -np.dot(dh_omega, dy)
        dq = np.dot(dh_omega, dx)
        dr = d2psi * np.dot(z_W, z_B)
        omega_dot = np.array([dp, dq, dr])

        tau1, tau2, tau3 = I @ omega_dot + np.cross(omega, np.dot(I, omega))

        torque = np.hstack([T, tau1, tau2, tau3])
        tor_mat = np.array(
            [[config.k, config.k, config.k, config.k],
             [0.0, -config.k * config.l, 0.0, config.k * config.l],
             [-config.k * config.l, 0.0, config.k * config.l, 0.0],
             [-config.b, config.b, -config.b, config.b],])
        u_motor = np.linalg.inv(tor_mat) @ torque
        u_control.append(u_motor)

    return np.array(u_control), np.array(state_ref), np.array(traj_ref)