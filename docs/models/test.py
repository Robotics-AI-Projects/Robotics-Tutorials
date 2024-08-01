import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

start_time = 0
end_time = 10
dt = 0.005
times = np.arange(start_time, end_time + dt, dt)

N = len(times)

x = np.array([0, 0, 10])
xdot = np.zeros(3)
theta = np.zeros(3)

deviation = 100
thetadot = np.deg2rad(2 * deviation * np.random.rand(3) - deviation)

m = 1.0
g = 9.81
k = 1.0
kd = 0.1
I = np.eye(3)
L = 0.25
b = 1.0

def thrust(inputs, k):
    return np.array([0, 0, k * np.sum(inputs)])

def torques(inputs, L, b, k):
    return np.array([
        L * k * (inputs[0] - inputs[2]),
        L * k * (inputs[1] - inputs[3]),
        b * (inputs[0] - inputs[1] + inputs[2] - inputs[3])
    ])

def rotation(angles):
    phi, theta, psi = angles
    Rz = np.array([
        [np.cos(psi), -np.sin(psi), 0],
        [np.sin(psi), np.cos(psi), 0],
        [0, 0, 1]
    ])
    Ry = np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(phi), -np.sin(phi)],
        [0, np.sin(phi), np.cos(phi)]
    ])
    return Rz @ Ry @ Rx

def acceleration(inputs, angles, xdot, m, g, k, kd):
    gravity = np.array([0, 0, -g])
    R = rotation(angles)
    T = R @ thrust(inputs, k)
    Fd = -kd * xdot
    return gravity + T / m + Fd

def angular_acceleration(inputs, omega, I, L, b, k):
    tau = torques(inputs, L, b, k)
    return np.linalg.inv(I) @ (tau - np.cross(omega, I @ omega))

def thetadot2omega(thetadot, theta):
    phi, theta, psi = theta
    T = np.array([
        [1, np.sin(phi) * np.tan(theta), np.cos(phi) * np.tan(theta)],
        [0, np.cos(phi), -np.sin(phi)],
        [0, np.sin(phi) / np.cos(theta), np.cos(phi) / np.cos(theta)]
    ])
    return T @ thetadot

def omega2thetadot(omega, theta):
    phi, theta, psi = theta
    T_inv = np.array([
        [1, np.sin(phi) * np.tan(theta), np.cos(phi) * np.tan(theta)],
        [0, np.cos(phi), -np.sin(phi)],
        [0, np.sin(phi) / np.cos(theta), np.cos(phi) / np.cos(theta)]
    ])
    return np.linalg.inv(T_inv) @ omega

def input(t):
    return np.array([400, 400, 400, 400])

x_history = []
theta_history = []

for t in times:
    i = input(t)

    omega = thetadot2omega(thetadot, theta)

    a = acceleration(i, theta, xdot, m, g, k, kd)
    omegadot = angular_acceleration(i, omega, I, L, b, k)

    omega = omega + dt * omegadot
    thetadot = omega2thetadot(omega, theta)
    theta = theta + dt * thetadot
    xdot = xdot + dt * a
    x = x + dt * xdot
    
    x_history.append(x.copy())
    theta_history.append(theta.copy())

x_history = np.array(x_history)
theta_history = np.array(theta_history)

def draw_drone(ax, position, orientation):
    arm_length = 0.005
    body = np.array([
        [-arm_length, 0, 0],
        [arm_length, 0, 0],
        [0, -arm_length, 0],
        [0, arm_length, 0]
    ])
    
    R = rotation(orientation)
    body_rotated = np.dot(body, R.T)

    body_translated = body_rotated + position

    for i in range(0, 4, 2):
        ax.plot([body_translated[i, 0], body_translated[i + 1, 0]],
                [body_translated[i, 1], body_translated[i + 1, 1]],
                [body_translated[i, 2], body_translated[i + 1, 2]], 'k-')

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)
ax.set_zlim(0, 20)

line, = ax.plot([], [], [], lw=2)
drone, = ax.plot([], [], [], 'ko', markersize=10)

def init():
    line.set_data([], [])
    line.set_3d_properties([])
    drone.set_data([], [])
    drone.set_3d_properties([])
    return line, drone

def update(frame):
    line.set_data(x_history[:frame, 0], x_history[:frame, 1])
    line.set_3d_properties(x_history[:frame, 2])

    drone.set_data(x_history[frame, 0], x_history[frame, 1])
    drone.set_3d_properties(x_history[frame, 2])

    ax.collections.clear()
    draw_drone(ax, x_history[frame], theta_history[frame])
    
    return line, drone

ani = FuncAnimation(fig, update, frames=N, init_func=init, blit=True)
plt.show()
