# Quadrotor Model and Simulation

In this tutorial we will derived quadrotor model and the following Python code of each equations. We gonna using ZYX Euler Angles convention.

$$
\zeta = \left [ x \; y \; z \right ]^T \in \R^3
$$

$$
\eta = \left [ \phi \; \theta \; \psi \right ]^T \in \R^3
$$

$$
\nu = \left [ u \; v \; w \right ]^T \in \R^3
$$

$$
\omega = \left [ p \; q \; r \right ]^T \in \R^3
$$

$$
\textbf{m}_B = \left [ m_x \; m_y \; m_z \right ]^T \in \R^3
$$

$$
\textbf{f}_B = \left [ f_x \; f_y \; f_z \right ]^T \in \R^3
$$

Where $\zeta$ and $\eta$ are linear and angular velocity from inertial frame to body frame, respectively. $\nu$ and $\omega$ are linear and angular velocity from body frame to inertial frame. $\textbf{f}_B$ and $\textbf{m}_B$ are total forces and moments.

## Mathematical Model

$$
\dot{\zeta} = R \cdot \nu \\
\dot{\eta} = T^{-1} \cdot \omega   
$$

where $T$ is a angular velocity transformation matrix from body frame to inertial frame,

$$
T = \begin{bmatrix}
1 & 0 & -\sin(\theta) \\
0 & \cos(\phi) & \sin(\phi)\sin(\theta) \\
0 & \sin(\phi) & \cos(\phi)\cos(\theta)
\end{bmatrix}
$$

$$
T^{-1} = \begin{bmatrix}
1 & \sin(\phi)\tan(\theta) & \cos(\phi) \\
0 & \cos(\phi) & -\sin(\theta) \\
0 & \frac{\sin(\phi)}{\cos(\theta)} & \frac{\cos(\phi)}{\cos(\theta)}
\end{bmatrix}
$$

 and $R$ is a Euler rotation matrix,

$$
R = \begin{bmatrix}
\cos\psi \cos\theta & \cos\psi \sin\theta \sin\phi - \sin\psi \cos\phi & \cos\psi \sin\theta \cos\phi + \sin\psi \sin\phi \\
\sin\psi \cos\theta & \sin\psi \sin\theta \sin\phi + \cos\psi \cos\phi & \sin\psi \sin\theta \cos\phi - \cos\psi \sin\phi \\
-\sin\theta & \cos\theta \sin\phi & \cos\theta \cos\phi
\end{bmatrix}
$$

## Moments

$$
\textbf{m}_B = I\cdot \dot{\omega} + \omega \times (I\cdot\omega_B) \\
\textbf{m}_B = \tau_B - \text{g}_a \\
I\cdot \dot{\omega} + \omega \times (I\cdot\omega_B) = \tau_B - \text{g}_a 
$$

so,

$$
\dot{\omega} =  I^{-1}(\tau_B - \omega \times (I \cdot \omega))
$$

where $I$ is a  inertia matrix, assume the quadrotor has a symmetric structure, so that 

$$
I = \begin{bmatrix}
I_{xx} & 0 & 0\\
0 & I_{yy} & 0\\
0 & 0 & I_{zz}
\end{bmatrix} \in \R^{3\times3}
$$

and $\tau_B$

$$
\tau_B = \begin{bmatrix} 
kl(\omega_{m,4}^2 - \omega_{m,2}^2)\\
kl(\omega_{m,3}^2 - \omega_{m,1}^2)\\
b(\omega_{m,1}^2 + \omega_{m,4}^2 - \omega_{m,1}^2-\omega_{m,3}^2)
\end{bmatrix}
$$

## Forces

$$
\textbf{f}_B = m(\omega \times \nu + \dot{\nu}) \\
\textbf{f}_B = - mgR^T + \textbf{T}_m \\
m(\omega \times \nu + \dot{\nu}) = - mgR^T + \textbf{T}_m
$$

so, 

$$
\dot{\nu} = - g R^T + \frac{\text{T}_B}{m}-\omega \times \nu
$$

given $\textbf{T}_B$

$$
\textbf{T}_B = \begin{bmatrix}
0\\
0\\
k\sum_{i=1}^4 \omega_{m,i}
\end{bmatrix}
$$

## State-Space Model

Given state vector :

$$
\textbf{x} = \begin{bmatrix}
\eta & \omega & \nu & \zeta
\end{bmatrix}^T \in \R^{12}
$$

$$
\begin{aligned}
\dot\phi  &=  p +r[\cos\phi\tan\theta]+[\sin\phi\tan\theta] \\
\dot\theta  &= q[\cos\phi]-r[\sin\phi]\\
\dot\psi &= r \frac{\cos\phi}{\cos\theta}+q\frac{\sin\phi}{\cos\theta}\\\
\dot{p} &= \frac{I_{yy}-I_{zz}}{I_{xx}}rq+\frac{\tau_x}{I_{xx}}\\
\dot{q} &= \frac{I_{zz}-I_{xx}}{I_{yy}}pr+\frac{\tau_y}{I_{yy}}\\
\dot{r} &= \frac{I_{xx}-I_{yy}}{I_{zz}}pq+\frac{\tau_z}{I_{zz}}\\
\dot{u} &= rv+qw+g[\sin\theta]\\
\dot{v} &= pw+ru-g[\sin\phi\cos\theta]\\
\dot{w} &= qu+pv-g[\cos\theta\cos\phi]\\
\dot{x} &= u[\cos\psi \cos\theta] - v[\cos\psi \sin\theta \sin\phi - \sin\psi \cos\phi] + w[\cos\psi \sin\theta \cos\phi + \sin\psi \sin\phi]\\
\dot{y} &= u[\sin\psi \cos\theta] + v[\sin\psi \sin\theta \sin\phi + \cos\psi \cos\phi] - w[\sin\psi \sin\theta \cos\phi - \cos\psi \sin\phi]\\
\dot{z} &= u[-\sin\theta] + v[\cos\theta \sin\phi]  w[\cos\theta \cos\phi]
\end{aligned}
$$
