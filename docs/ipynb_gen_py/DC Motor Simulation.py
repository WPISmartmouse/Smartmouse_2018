
# coding: utf-8

# # DC Motor Simulation

# Look at the dynamics_model.pdf for the explanation behind this code.

# In[3]:

import numpy as np
import matplotlib.pyplot as plt


# In[33]:

theta_dot = 0
i = 0

# constants
J = 0.01
b = 0.1
K = 0.01
R = 1
L = 0.5

# input
def V(t):
    return 1

# simulate T seconds
T = 5
dt = 0.01
theta_dots = []
ts = np.arange(0, T, dt)
for t in ts:
    A = np.array([[-b/J,K/J],[-K/L,-R/L]])
    B = np.array([0, 1/L])
    x = np.array([theta_dot, i])
    w = V(t)
    x_dot = A@x+B*w
    theta_dot += (x_dot[0] * dt)
    i += (x_dot[1] * dt)
    theta_dots.append(theta_dot)
#     print(A, x, B, w, "->", x_dot)

plt.plot(ts, theta_dots)
plt.ylabel("Speed (rad/s)")
plt.xlabel("Time (seconds)")
plt.show()


# In[ ]:



