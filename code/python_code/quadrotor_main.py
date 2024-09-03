from helper import plot_quadrotor
from simulator import quadrotor
from trajectory.diff_flat_qc import trajectory_ref 
import numpy as np
from tqdm import tqdm
from colorama import Fore, Back, Style

def lemin_trajectory(t, a=2, tfac=1.2):
  x = (a * np.cos(np.sqrt(a) * t/ tfac))
  y = (a * np.sin(np.sqrt(2) * t / tfac) * np.cos(np.sqrt(2) * t / tfac))
  z = 0.0
  phi = 0.0
  return np.array([x, y, z, phi])

target = 5.5
dt = 0.01
t_ref = np.arange(0.0, target + dt, dt)

config = quadrotor.QuadConfig()
u_control, state_ref, traj_ref = trajectory_ref(t_ref, config, lemin_trajectory)
qc = quadrotor.Quadrotor(config=config, initial_state=state_ref[0])
plot_quad = plot_quadrotor.Plot_Quadrotor(init_state=state_ref[0], trajectory_ref=traj_ref, show_animation=True)

state_history = np.zeros((len(t_ref), 12))
control_history = np.zeros((len(t_ref), 4))

print("\n  " + Fore.WHITE + Back.GREEN + '\033[1m\t\tSTART SIMULATION\t\t\033[1m'+Style.RESET_ALL+"\n")

for i, t in tqdm(enumerate(t_ref), total=len(t_ref), colour="red"):
    state = qc.update_state(u_control[i], dt)
    plot_quad.update_pose(state)
    state_history[i, :] = state
    control_history[i, :] = u_control[i]

plot_quad.save_animation(filename='../media/qc_animation.mp4', interval=50, fps=20)




