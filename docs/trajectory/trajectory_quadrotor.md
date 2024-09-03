# Trajectory Control of Quadrotor

State of quadrotor dynamics donated by :

$$
\textbf{x} = \begin{bmatrix}
x \; y \; z \; \phi \; \theta \; \psi \; u \; v \; w \; p \; q \; r
\end{bmatrix}^T \in \R^{12}
$$

Dengan Newton-Euler, diperoleh akselerasi linear 

$$
\begin{equation}
m \ddot{r} =-mg \textbf{z}_W + T \textbf{z}_B
\end{equation} 
$$

$r$ adalah posisi dari CoG $[x \; y \; z]$, $T$ adalah thrust dari rotor

Akselerasi angular :

$$
\begin{equation}
\dot{\omega} =  I^{-1}(\tau_B - \omega \times (I \cdot \omega))
\end{equation}
$$

$$
\omega_{BW} = px_B+qy_B+rz_B
$$



## Differential Flatness

Dinamika quadrotor dengan dengan 4 input merupakan deferensial pada bidang datar. Sehingga,

$$
\sigma = \left[ x \; y \; z \; \psi \right]^T
$$

untuk mendapatkan referensi tersebut dibutuhkan derivative 4 tingkat, ***velocity***, ***acceleration***, ***jerk***, dan ***snap***.

Dari persamaan **(1)**, dapat diperoleh unit vektor $\textbf{z}_B$,

$$
\begin{equation}
\frac{T}{m} \textbf{z}_B = \ddot{r} + g \\
\end{equation}
$$

untuk mendapatkan arah dari unit vektor $\textbf{z}_B$,

$$
\begin{equation}
\textbf{z}_B = \frac{\ddot{\sigma}_{pos} + g}{||\ddot{\sigma}_{pos} + g||}
\end{equation}
$$

$T$ dan $m$ tidak berpengaruh terhadap arah dari $\textbf{z}_B$, hanya mempengaruhi skalanya.

Unit vektor $[\textbf{x}_C \; \textbf{y}_C \; \textbf{z}_C]$ pada CoG, unit vektor pada x-axis dapat peroleh dengan

$$
\begin{equation}
\textbf{x}_C = [\cos \psi \; \sin\psi\; 0]^T
\end{equation}
$$

Persamaan **(5)** merepresentasikan heading pada *xy-plane*. 

$$
\begin{equation}
\begin{aligned}
\textbf{y}_B = \frac{\textbf{z}_B \times \textbf{x}_C}{||\textbf{z}_B \times \textbf{x}_C||}
\\
\textbf{x}_B = \textbf{y}_B \times \textbf{z}_B
\end{aligned}
\end{equation}
$$

Pada *cross-product* pada persamaan $\textbf{y}_B$ merupakan operasi untuk mendapatkan vektor yang tegak lurus terhadap keduanya, begitu juga pada persamaan $\textbf{x}_B$. Normalisasi dengan membagikan terhadap magnitude-nya untuk mendapatkan unit vektor. Dengan asumsi $\textbf{x}_C \times \textbf{z}_B \neq 0$, yang mengindikasikan kedua vektor tidak paralel.

Matriks rotasi dari body-frame ke world-frame dengan menggunakan unit vektor pada body-frame,

$$
\begin{equation}
R_{BW} = [\textbf{x}_B \; \textbf{y}_B \; \textbf{z}_B]
\end{equation}
$$

Dengan menggunakan turunan pertama dari permsamaan **(1)**, 

$$
\begin{equation}
\begin{aligned}
m \ddot{r} = m\textbf{a} =-mg \textbf{z}_W + T \textbf{z}_B \\
m\dot{\textbf{a}} = \dot{F} \textbf{z}_B + T \dot{z_B}
\end{aligned}
\end{equation} 
$$

Dengan $\dot{z}_B = \omega_{BW} \times z_B$,

$$
\begin{equation}
\begin{aligned}
m\dot{\textbf{a}} = \dot{T} \textbf{z}_B + T (\omega_{BW} \times z_B) \\
m\dot{\textbf{a}} = \dot{T} \textbf{z}_B + \omega_{BW} \times Tz_B
\end{aligned}
\end{equation} 
$$

Dengan $\dot{T} = \textbf{z}_B \cdot m \dot{\textbf{a}}$, maka

$$
\begin{equation}
\begin{aligned}
m\dot{\textbf{a}} = (\textbf{z}_B \cdot m \dot{\textbf{a}}) \textbf{z}_B + \omega_{BW} \times Tz_B\\
\omega_{BW} \times Tz_B = m(\dot{\textbf{a}} - 
\textbf{z}_B \cdot \dot{\textbf{a}}) \textbf{z}_B \\
\omega_{BW} \times z_B = \frac{m}{T}(\dot{\textbf{a}}-(z_B\cdot \dot{\textbf{a}})z_B) 
\end{aligned}
\end{equation} 
$$

Sehingga dapat menyelesaikan $p, q, $ dan $r$

$$
\begin{equation}
\begin{aligned}
p= -(\omega_{BW} \times z_B)\cdot \textbf{y}_B\\
q= (\omega_{BW} \times z_B)\cdot \textbf{x}_B\\
r = \ddot{\psi}\textbf{z}_W \cdot \textbf{z}_B
\end{aligned}
\end{equation}
$$

Turunan kedua persamaan **(1)** merupakan turunan pertama dari persamaan **(9)**,

$$
\begin{equation}
\begin{aligned}
m\ddot{\textbf{a}} = \ddot{T}\textbf{z}_B + 
\dot{T}\dot{\textbf{z}}_B + 
\dot{\omega}_{BW} \times T\textbf{z}_{B} + 
\omega_{BW} \times \dot{T}z_B+ \omega_{BW} \times T\dot{z_B}\\
\end{aligned}
\end{equation}
$$

Dan $\ddot{T} = m(\dot{\textbf{z}}_B \cdot \dot{\textbf{a}} + \textbf{z}_B \cdot \ddot{\textbf{a}})$

$$
m\ddot{a} -\ddot{T}z_B -\omega_{BW}\times \dot{T}z_B = 
\dot{\omega}_{BW}\times Tz_B + \omega_{BW} \times T\dot{z}_{B} + \dot{T}\dot{z}_B
$$






