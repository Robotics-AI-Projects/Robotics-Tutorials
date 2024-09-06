from helper import plot_quadrotor
from simulator import quadrotor
from trajectory.differential_flatness_qc import trajectory_ref 
import numpy as np
from tqdm import tqdm
from colorama import Fore, Back, Style
import matplotlib.pyplot as plt
from data_driven.gen_func import GenerateFunction
from data_driven.wynda import WyNDA





# Quadcopter configuration
# Configuration for a quadcopter.
# Input :
#   Ixx : Inertia of x-axis
#   Iyy : Inertia of y-axis
#   Izz : Inertia of z-axis
#   k   : Friction constant
#   b   : Drag constant
#   l   : Length of arm
#   m   : Mass of quadcopter
#   g   : Gravity constant
# Output : None

# config = {
#     "Ixx": 4.856e-3,
#     "Iyy": 4.856e-3,
#     "Izz": 8.801e-3,
#     "b": 1.14e-8,
#     "k": 2.980e-6,
#     "l": 0.225,
#     "m": 0.468,
#     "g": 9.81,
# }

config = {
    "Ixx": 1.0,
    "Iyy": 1.0,
    "Izz": 2.0,
    "k": 1.0,
    "b": 0.5,
    "l": 1.0 / 3,
    "m": 2.0,
    "g": 9.81,
}

def base_func(state, config):
  x, y, z = state[:3]
  phi, theta, psi = state[3:6]
  u, v, w = state[6:9]
  p, q, r = state[9:12]

  phi_dot = p
  theta_dot = q
  psi_dot = r

  p_dot = 0
  q_dot = 0
  r_dot = 0

  u_dot = config["g"]*theta
  v_dot = config["g"]*phi
  w_dot = 0

  x_dot = u
  y_dot = v
  z_dot = w

  return np.array([x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot, u_dot, v_dot, w_dot, p_dot, q_dot, r_dot])

def basis_func(state, control):
  u1, u2, u3, u4 = control
  p, q, r = state[9:12]
  mu = np.array([1, u1, u2, u3, u4, p, q, r, p*q, p*r, q*r, p*q*r])
  return mu

def lemin_trajectory(t, a=2, tfac=1.2):
  x = (a * np.cos(np.sqrt(a) * t/ tfac))
  y = (a * np.sin(np.sqrt(a) * t / tfac) * np.cos(np.sqrt(a) * t / tfac))
  z = 0.0
  phi = 0.0
  return np.array([x, y, z, phi])

def circle_trajectory(t):
    """Circular trajectory in the x-y plane"""
    tfac = 1.2
    x = 2.0 * np.cos(np.sqrt(2) * t / tfac)
    y = 2.0 * np.sin(np.sqrt(2) * t / tfac)
    z = 0.0

    phi = 0.0
    return np.array([x, y, z, phi])

target = 5.5
dt = 0.01
t_ref = np.arange(0.0, target + dt, dt)

u_control, state_ref, traj_ref = trajectory_ref(t_ref, config, circle_trajectory)
qc = quadrotor.Quadrotor(config=config, initial_state=state_ref[0])
plot_quad = plot_quadrotor.Plot_Quadrotor(init_state=state_ref[0], 
                                          trajectory_ref=traj_ref, 
                                          xlimit=[-3, 3],
                                          ylimit=[-3, 3],
                                          zlimit=[-3, 3],
                                          show_animation=False)
wynda = WyNDA(12, 144, state_ref[0])
gen_func = GenerateFunction(12)

state_history = np.zeros((len(t_ref), 12))
wynda_states = np.zeros((len(t_ref), 12))
control_history = np.zeros((len(t_ref), 4))

print("\n  " + Fore.WHITE + Back.GREEN + '\033[1m\t\tSTART SIMULATION\t\t\033[1m'+Style.RESET_ALL+"\n")

for i, t in tqdm(enumerate(t_ref), total=len(t_ref), colour="red"):
    state = qc.update_state(u_control[i], dt)
    base = base_func(state, config)
    basis = basis_func(state, u_control[i])
    Phi = gen_func.custom(basis)
    wynda_state, wynda_params = wynda.run(state, Phi, dt, base)
    # plot_quad.update_pose(state)
    state_history[i, :] = state
    state_history[i, :] = wynda_state
    control_history[i, :] = u_control[i]

# plot_quad.save_animation(filename='../media/qc_animation.mp4', interval=50, fps=20)




