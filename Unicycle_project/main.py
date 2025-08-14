from unicycle import Unicycle
from controllers import PID, LQRController, MPCController
import numpy as np
import matplotlib.pyplot as plt
import math

EPS = 0.02
dt, T = 0.02, 20
goal = (2.0, 1.5)
v_min, kv, v_max = 0.2, 0.8, 0.8
u_min, u_max = -2.0, 2.0
kp, ki, kd= 2.5, 0.0, 0.1
N = 10

input_mode = input("Please enter the controller interested - PID or LQR or MPC: ").upper()
if input_mode not in ("PID", "LQR", "MPC"):
    print("Entered controller does not exist, continuing with PID")
    input_mode = "PID"

t = np.arange(0.0, T, dt)
uni_bot = Unicycle(x = 0.0, y = 0.0, th = 0.0)
pid_controller = PID(kp=kp, ki=ki, kd=kd, dt=dt, u_min= u_min, u_max= u_max, v_max= v_max, kv= kv)
lqr_controller = LQRController(v0 = 0.2, dt = dt, v_min = v_min, v_max = v_max,  w_min= u_min, w_max= u_max)
mpc_controller = MPCController(v0 = 0.2, dt = dt, N = 10)
xs, ys, ths = [], [], []

if input_mode == 'PID':
    controller = pid_controller
elif input_mode == 'LQR':
    controller = lqr_controller
else:
    controller = mpc_controller

for _ in t:
    if math.hypot(goal[0] - uni_bot.x, goal[1] - uni_bot.y) < EPS:
        v, w = 0.0, 0.0
    else:
        v, w = controller.step(uni_bot.x, uni_bot.y, uni_bot.th, goal)
    uni_bot.model(v, w, dt)
    xs.append(uni_bot.x)
    ys.append(uni_bot.y)
    ths.append(uni_bot.th)

plt.figure()
plt.plot(xs, ys, label="Path")
plt.plot(goal[0], goal[1], 'ro', label="Goal")
plt.axis("equal")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.title(f"Unicycle Path to Goal - {input_mode}")
plt.legend()
plt.grid(True)
plt.show()