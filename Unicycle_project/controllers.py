import numpy as np
import casadi as ca
import math
Q_WEIGHT_MATRIX = np.diag([3.0, 3.0, 1.0])
R_WEIGHT_MATRIX = np.diag([0.6, 0.1])
MAX_ITER = 500
TOLERANCE = 1e-10
NO_OF_STEPS = 10

def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

def rot_errors(x, y, th, gx, gy):
    dx, dy = gx - x, gy - y
    th_g = np.arctan2(dy, dx)
    c, s = np.cos(th_g), np.sin(th_g)
    ex = c * dx + s * dy
    ey = -s * dx + c * dy
    eth = (th_g - th + np.pi) % (2 * np.pi) - np.pi
    return ex, ey, eth

class PID:
    def __init__(self, kp,ki,kd,dt, u_max = None, u_min= None, v_max= None, kv= None):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.dt = dt
        self.v_max = v_max
        self.Kv = kv
        self.u_max = u_max
        self.u_min = u_min
        self.i_max, self.i_min = 1e6,-1e6
        self.e_prev, self.I = 0.0, 0.0

    def omega(self,e):
        d = (e - self.e_prev)/ self.dt
        u_unsaturated = self.Kp * e + self.Ki * ( self.I + e*self.dt) + self.Kd * d
        u = u_unsaturated
        if self.u_min is not None: u = max(self.u_min, u)
        if self.u_max is not None: u = min(self.u_max, u)

        #anti_windup
        if (u == u_unsaturated) or (self.Ki == 0) or ((u == self.u_max and e < 0) or (u == self.u_min and e > 0)):
            self.I = max(self.i_min, min(self.i_max, self.I + e * self.dt))
        self.e_prev = e
        return u

    def step(self, x, y, th, goal):
        gx, gy = goal
        dx, dy = gx - x, gy - y
        rho = math.hypot(dx, dy)
        thg = math.atan2(dy, dx)
        alpha = wrap(thg - th)

        v = min(self.v_max, self.Kv * rho)
        w = self.omega(alpha)
        return v,w

class LQRController:
    def __init__(self, v0: float, dt: float, v_min, v_max, w_min, w_max):

        self.v0 = float(v0)
        self.dt = float(dt)
        self.v_min, self.v_max = v_min, v_max
        self.w_min, self.w_max= w_min, w_max

        A_C = np.array([[0.0, 0.0, self.v0],
                        [0.0, 0.0, -self.v0],
                        [0.0, 0.0, 0.0]], dtype=float)
        B_C = np.array([[-1.0, 0.0],
                        [0.0, self.v0],
                        [0.0, -1.0]], dtype=float)

        self.A = np.eye(3) + self.dt * A_C
        self.B = self.dt * B_C
        self.Q = Q_WEIGHT_MATRIX
        self.R = R_WEIGHT_MATRIX
        self.K, self.P = self.d_lqr(A = self.A, B = self.B, Q = self.Q, R = self.R, max_iter = MAX_ITER, tol= TOLERANCE)

    @staticmethod
    def d_lqr(A, B, Q, R, max_iter = MAX_ITER, tol = TOLERANCE):
        P = Q.copy()
        for _ in range(max_iter):
            BT_P = B.T @ P
            S = R + BT_P @ B
            K = np.linalg.solve(S, BT_P @ A)
            Pn = Q + A.T @ P @ (A - B @ K)
            if np.linalg.norm(Pn - P, ord='fro') < tol:
                P = Pn
                break
            P = Pn
        K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
        return K, P

    def get_k(self):
        return self.K.copy()

    def step(self, x: float, y: float, th: float,
             goal: tuple[float,float]) -> tuple[float,float]:
        gx, gy = goal
        ex, ey, eth = rot_errors(x, y, th, gx, gy)

        e = np.array([[ex], [ey], [eth]])
        u = -self.K @ e
        dv, w = float(u[0,0]), float(u[1,0])
        v = self.v0 + dv
        v = max(self.v_min, min(self.v_max, v))
        w = max(self.w_min, min(self.w_max, w))
        return v, w

class MPCController:
    def __init__(self, v0: float, dt: float, N: int = NO_OF_STEPS):

        self.v0 = v0
        self.dt = dt
        self.N = N
        self.nx = 3
        self.nu = 2

        self.Q = Q_WEIGHT_MATRIX
        self.R = R_WEIGHT_MATRIX

        A_C = np.array([[0.0, 0.0, self.v0],
                        [0.0, 0.0, -self.v0],
                        [0.0, 0.0, 0.0]], dtype=float)
        B_C = np.array([[-1.0, 0.0],
                        [0.0, self.v0],
                        [0.0, -1.0]], dtype=float)

        self.A = np.eye(3) + self.dt * A_C
        self.B = self.dt * B_C + 0.5 * (self.dt**2) * (A_C @ B_C)

        x = ca.SX.sym('x', self.nx)
        u = ca.SX.sym('u', self.nu)
        x_next = ca.mtimes(self.A, x) + ca.mtimes(self.B, u)
        f = ca.Function('f', [x, u], [x_next])

        U = ca.SX.sym('U', self.nu, N)
        X = ca.SX.sym('X', self.nx, N+1)
        X0 = ca.SX.sym('X0', self.nx)

        cost = 0
        g = []
        g.append(X[:, 0] - X0)

        for k in range(N):
            cost += ca.mtimes([X[:, k].T, self.Q, X[:, k]]) + ca.mtimes([U[:, k].T, self.R, U[:, k]])
            g.append(X[:, k+1] - f(X[:, k], U[:, k]))

        cost += ca.mtimes([X[:, N].T, self.Q, X[:, N]])
        g = ca.vertcat(*g)

        nlp = {
            'x': ca.vertcat(ca.reshape(U, -1, 1), ca.reshape(X, -1, 1)),
            'f': cost,
            'g': g,
            'p': X0
        }

        opts = {'ipopt.print_level': 0, 'print_time': 0}
        self.solver = ca.nlpsol('solver', 'ipopt', nlp, opts)

    def step(self, x: float, y: float, th: float,
             goal: tuple[float, float]) -> tuple[float, float]:

        ex, ey, eth = rot_errors(x , y, th, goal[0], goal[1])
        e0 = np.array([ex, ey, eth])
        sol = self.solver(
            lbx= -ca.inf, ubx= ca.inf,
            lbg= 0, ubg= 0,
            p = e0
        )

        u0_opt = np.array(sol['x'][:self.nu]).flatten()
        dv, w = u0_opt
        v = self.v0 + dv
        return v,w