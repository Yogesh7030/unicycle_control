# Unicycle Robot Path Tracking — PID, LQR, MPC

This repository implements a **Unicycle kinematic model** with three controllers for path tracking:  
- **PID** (Proportional-Integral-Derivative)  
- **LQR** (Linear Quadratic Regulator)  
- **MPC** (Model Predictive Control — without actuator constraints yet)  

---

## 1. Unicycle Model

The **unicycle kinematic equations** (pose \((x, y, \theta)\), control inputs: linear velocity \(v\) and angular velocity \(\omega\)):

$$
\dot{x} = v\cos\theta,\qquad
\dot{y} = v\sin\theta,\qquad
\dot{\theta} = \omega.
$$

Discrete-time update (Euler integration):

$$
\x_{k+1} = x_k + v_k \cos(\theta_k) \cdot \Delta t\, \qquad
\y_{k+1} = y_k + v_k \sin(\theta_k) \cdot \Delta t\, \qquad
\theta_{k+1} = \theta_k + \omega_k \cdot \Delta t\
$$

---

## 2. PID Controller

We use a **heading-based PID** controller:  
- \(v\) is proportional to **distance to goal**  
- \(\omega\) is computed from a PID on the **heading error**

**Heading error:**
\[
\alpha = \text{wrap}(\theta_g - \theta)
\]
where:
\[
\theta_g = \arctan2(y_g - y, x_g - x)
\]

**Linear velocity command:**
\[
v = \min(v_{\text{max}}, k_v \cdot \rho)
\]
where:
\[
\rho = \sqrt{(x_g - x)^2 + (y_g - y)^2}
\]

**Angular velocity command:**
\[
\omega = k_p \alpha + k_i \int \alpha \, dt + k_d \frac{d\alpha}{dt}
\]

### Parameters to Tune:
| Variable | Meaning | Typical Range |
|----------|---------|---------------|
| `kp` | Proportional gain | 1.0 – 4.0 |
| `ki` | Integral gain | 0.0 – 0.5 |
| `kd` | Derivative gain | 0.0 – 0.5 |
| `kv` | Linear speed gain | 0.1 – 1.0 |
| `v_max` | Max forward speed | 0.1 – 1.0 m/s |
| `u_max`, `u_min` | Angular velocity limits | ±2 rad/s |

---

## 3. LQR Controller

We linearize the **unicycle tracking error dynamics** around a nominal velocity \(v_0\) in the goal-aligned frame:

\[
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
\]

Where:
\[
A_c =
\begin{bmatrix}
0 & 0 & v_0 \\
0 & 0 & -v_0 \\
0 & 0 & 0
\end{bmatrix}
,\quad
B_c =
\begin{bmatrix}
-1 & 0 \\
0 & v_0 \\
0 & -1
\end{bmatrix}
\]

Discrete-time form:
\[
A = I + \Delta t \cdot A_c
\]
\[
B = \Delta t \cdot B_c
\]

**LQR gain \(K\)** is found by solving the Discrete Algebraic Riccati Equation:
\[
K = (R + B^T P B)^{-1} (B^T P A)
\]

Control law:
\[
u = -K e
\]
with:
\[
u = 
\begin{bmatrix}
\Delta v \\ \omega
\end{bmatrix}
\]
and \(v = v_0 + \Delta v\).

### Parameters to Tune:
| Variable | Meaning |
|----------|---------|
| `Q` | State error weighting matrix |
| `R` | Control effort weighting matrix |
| `v0` | Nominal forward velocity |
| `v_min`, `v_max` | Speed limits |
| `w_min`, `w_max` | Angular velocity limits |

---

## 4. MPC Controller (Linear, No Constraints)

We use the **same linearized model** as LQR, but predict over a finite horizon \(N\):

\[
\min_{U} \sum_{k=0}^{N-1} \left( e_k^T Q e_k + u_k^T R u_k \right) + e_N^T Q e_N
\]
subject to:
\[
e_{k+1} = A e_k + B u_k
\]
\[
e_0 = \text{current error}
\]

Where:
- \(U = [u_0, u_1, \dots, u_{N-1}]\)  
- At each step, we solve for the **optimal sequence**, but apply only the **first control** (receding horizon).

**Note:**  
- No actuator constraints yet (no limits on \(\Delta v, \omega\) in solver).  
- Adding constraints would turn this into a **constrained QP** or **nonlinear MPC**.

### Parameters to Tune:
| Variable | Meaning |
|----------|---------|
| `Q`, `R` | Weight matrices |
| `N` | Prediction horizon length |
| `v0` | Nominal forward velocity |

---

## 5. How to Run

When Prompted:
Please enter the controller interested - PID or LQR or MPC:

```bash
pip install -r requirements.txt
python main.py
