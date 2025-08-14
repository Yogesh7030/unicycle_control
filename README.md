# Unicycle Robot Path Tracking — PID, LQR, MPC

This repository implements a **Unicycle kinematic model** with three controllers for path tracking:  
- **PID** (Proportional-Integral-Derivative)  
- **LQR** (Linear Quadratic Regulator)  
- **MPC** (Model Predictive Control — currently without actuator constraints)  

---

## 1. Unicycle Model

The **unicycle kinematic equations** (pose \((x, y, \theta)\), control inputs: linear velocity \(v\) and angular velocity \(\omega\)) are:

Continuous-time:
$$
\dot{x} = v \cos\theta, \quad
\dot{y} = v \sin\theta, \quad
\dot{\theta} = \omega
$$

Discrete-time (Euler integration):
$$
\begin{aligned}
x_{k+1} &= x_k + v_k \cos(\theta_k) \cdot \Delta t \\
y_{k+1} &= y_k + v_k \sin(\theta_k) \cdot \Delta t \\
\theta_{k+1} &= \theta_k + \omega_k \cdot \Delta t
\end{aligned}
$$

---

## 2. PID Controller

We use a **heading-based PID** controller:  

- \( v \) is proportional to **distance to goal**  
- \( \omega \) is computed from a PID on the **heading error**

**Heading error:**
$$
\begin{aligned}
\alpha &= \text{wrap}(\theta_g - \theta) \\
\theta_g &= \arctan2(y_g - y, \; x_g - x)
\end{aligned}
$$

**Linear velocity command:**
$$
\begin{aligned}
\rho &= \sqrt{(x_g - x)^2 + (y_g - y)^2} \\
v &= \min(v_{\text{max}},\; k_v \cdot \rho)
\end{aligned}
$$

**Angular velocity command:**
$$
\omega = k_p \alpha + k_i \int \alpha \, dt + k_d \frac{d\alpha}{dt}
$$

**Parameters to Tune:**
| Variable | Meaning | Typical Range |
|----------|---------|---------------|
| `kp` | Proportional gain | 1.0 – 4.0 |
| `ki` | Integral gain | 0.0 – 0.5 |
| `kd` | Derivative gain | 0.0 – 0.5 |
| `kv` | Linear speed gain | 0.1 – 1.0 |
| `v_max` | Max forward speed (m/s) | 0.1 – 1.0 |
| `u_max`, `u_min` | Angular velocity limits (rad/s) | ±2 |

---

## 3. LQR Controller

We linearize the **tracking error dynamics** around a nominal velocity \( v_0 \) in the goal-aligned frame:

Continuous-time error dynamics:
$$
\begin{bmatrix}
\dot{e_x} \\[2pt]
\dot{e_y} \\[2pt]
\dot{e_\theta}
\end{bmatrix}
=
A_c
\begin{bmatrix}
e_x \\ e_y \\ e_\theta
\end{bmatrix}
+
B_c
\begin{bmatrix}
\Delta v \\ \omega
\end{bmatrix}
$$

Where:
$$
A_c =
\begin{bmatrix}
0 & 0 & v_0 \\
0 & 0 & -v_0 \\
0 & 0 & 0
\end{bmatrix}, \quad
B_c =
\begin{bmatrix}
-1 & 0 \\
0 & v_0 \\
0 & -1
\end{bmatrix}
$$

Discrete-time form:
$$
A = I + \Delta t \cdot A_c, \quad
B = \Delta t \cdot B_c
$$

**LQR gain \(K\)** is computed by solving the Discrete Algebraic Riccati Equation (DARE):
$$
K = (R + B^T P B)^{-1} (B^T P A)
$$

Control law:
$$
u = -K e, \quad
u =
\begin{bmatrix}
\Delta v \\ \omega
\end{bmatrix}, \quad
v = v_0 + \Delta v
$$

**Parameters to Tune:**
| Variable | Meaning |
|----------|---------|
| `Q` | State error weight |
| `R` | Control effort weight |
| `v0` | Nominal forward velocity |
| `v_min`, `v_max` | Speed limits |
| `w_min`, `w_max` | Angular velocity limits |

---

## 4. MPC Controller (Linear, No Constraints)

We use the **same linearized model** as LQR, but optimize over a finite prediction horizon \(N\):

Optimization problem:
$$
\min_{U} \sum_{k=0}^{N-1} \left( e_k^T Q e_k + u_k^T R u_k \right) + e_N^T Q e_N
$$
subject to:
$$
e_{k+1} = A e_k + B u_k, \quad e_0 = \text{current error}
$$

Where:
- \( U = [u_0, u_1, \dots, u_{N-1}] \)  
- At each time step, solve for the optimal sequence and apply only the **first control** (receding horizon).

**Note:**  
- Currently no actuator constraints (limits on \(\Delta v, \omega\))  
- Adding constraints turns this into a **constrained MPC** problem.

**Parameters to Tune:**
| Variable | Meaning |
|----------|---------|
| `Q`, `R` | Weight matrices |
| `N` | Prediction horizon length |
| `v0` | Nominal forward velocity |

---

## 5. Example Outputs

| PID Controller | LQR Controller | MPC Controller |
|----------------|----------------|----------------|
| ![PID Result](pid_controller_result.png) | ![LQR Result](lqr_controller_result.png) | ![MPC Result](mpc_controller_result.png) |

---

## 6. How to Run

Install dependencies:
```bash
pip install -r requirements.txt
python main.py
Please enter the controller interested - PID or LQR or MPC:
