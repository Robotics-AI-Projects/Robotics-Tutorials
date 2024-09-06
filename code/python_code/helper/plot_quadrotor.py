"""
Modified quadrotor animation code from PythonRobotics by Daniel Ingram (daniel-s-ingram)

Author  : Rafi Darmawan (drmwnnrafi)
"""

import matplotlib.animation as animation
from numpy import cos, sin
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
from colorama import Fore, Back, Style

class Plot_Quadrotor():
    def __init__(self, 
                 init_state:np.array=np.zeros(12), 
                 trajectory_ref:np.array=None,
                 size:int=1, 
                 show_animation:bool=False, 
                 xlimit:list=None, 
                 ylimit:list=None, 
                 zlimit:list=None,):
        self.x, self.y, self.z = init_state[0:3]
        self.roll, self.pitch, self.yaw = init_state[3:6]
        self.size = size
        self.p1 = np.array([size / 2, 0, 0, 1]).T
        self.p2 = np.array([-size / 2, 0, 0, 1]).T
        self.p3 = np.array([0, size / 2, 0, 1]).T
        self.p4 = np.array([0, -size / 2, 0, 1]).T

        self.x_data = []
        self.y_data = []
        self.z_data = []
        self.traj_ref = trajectory_ref
        self.wing1 = []
        self.wing2 = []
        self.show_animation = show_animation

        self.xlim = xlimit
        self.ylim = ylimit
        self.zlim = zlimit

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.anim = None
        self.update_pose(init_state)

    def update_pose(self, state):
        self.x, self.y, self.z = state[0:3]
        self.roll, self.pitch, self.yaw = state[3:6]
        self.x_data.append(self.x)
        self.y_data.append(self.y)
        self.z_data.append(self.z)
        self.plot()

    def transformation_matrix(self):
        x = self.x
        y = self.y
        z = self.z
        roll = self.roll
        pitch = self.pitch
        yaw = self.yaw
        return np.array(
            [[cos(yaw) * cos(pitch), -sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll), sin(yaw) * sin(roll) + cos(yaw) * sin(pitch) * cos(roll), x],
             [sin(yaw) * cos(pitch), cos(yaw) * cos(roll) + sin(yaw) * sin(pitch) * sin(roll), -cos(yaw) * sin(roll) + sin(yaw) * sin(pitch) * cos(roll), y],
             [-sin(pitch), cos(pitch) * sin(roll), cos(pitch) * cos(roll), z]
             ])

    def plot(self):
        T = self.transformation_matrix()

        p1_t = np.matmul(T, self.p1)
        p2_t = np.matmul(T, self.p2)
        p3_t = np.matmul(T, self.p3)
        p4_t = np.matmul(T, self.p4)

        wing1 = np.array([[p1_t[0], p2_t[0]], [p1_t[1], p2_t[1]], [p1_t[2], p2_t[2]]])
        wing2 = np.array([[p3_t[0], p4_t[0]], [p3_t[1], p4_t[1]], [p3_t[2], p4_t[2]]])

        self.wing1.append(wing1)
        self.wing2.append(wing2)

        if self.show_animation :
            self.ax.cla()
            self.ax.plot3D(wing1[0], wing1[1], wing1[2], 'r-')
            self.ax.plot3D(wing2[0], wing2[1], wing2[2], 'b-')

            self.ax.plot3D(self.x_data, self.y_data, self.z_data, color="green", linestyle='--',)
            self.ax.plot3D(self.traj_ref[:, 0], self.traj_ref[:, 1], self.traj_ref[:, 2], color="cyan", linestyle='-', alpha=0.5)

            if self.xlim :
                self.ax.set_xlim(self.xlim[0], self.xlim[1])
            if self.ylim :
                self.ax.set_ylim(self.ylim[0], self.ylim[1])
            if self.zlim :
                self.ax.set_zlim(self.zlim[0], self.zlim[1])
            plt.pause(0.001)

    def save_animation(self, filename='quadcopter_animation.mp4', interval=100, fps=10, dpi=300):
        print("\n" + Fore.WHITE + Back.GREEN + '\t\t\033[1mSAVE ANIMATION as mp4\t\t\033[1m'+Style.RESET_ALL+"\n")

        bar = tqdm(total=len(self.x_data), colour="yellow")
        ani = animation.FuncAnimation(self.fig, self._animate, frames=len(self.x_data),
                                      interval=interval, repeat=False)
        ani.save(filename, writer='ffmpeg', fps=fps, dpi=dpi, progress_callback=lambda i, n: bar.update(1))

    def _animate(self, i):
        self.ax.cla()
        if i >= len(self.wing1):
            return
        
        wing1 = self.wing1[i]
        wing2 = self.wing2[i]

        self.ax.plot3D(wing1[0], wing1[1], wing1[2], 'r-')
        self.ax.plot3D(wing2[0], wing2[1], wing2[2], 'b-')

        self.ax.plot3D(self.x_data[:i], self.y_data[:i], self.z_data[:i], color="green", linestyle='--',)
        self.ax.plot3D(self.traj_ref[:, 0], self.traj_ref[:, 1], self.traj_ref[:, 2], color="cyan", linestyle='-', alpha=0.5)

        if self.xlim :
            self.ax.set_xlim(self.xlim[0], self.xlim[1])
        if self.ylim :
            self.ax.set_ylim(self.ylim[0], self.ylim[1])
        if self.zlim :
            self.ax.set_zlim(self.zlim[0], self.zlim[1])