
# coding: utf-8

# # DC Motor Simulation

# Look at the dynamics_model.pdf for the explanation behind this code.

# In[1]:


import numpy as np
import matplotlib.pyplot as plt
plt.style.use("./smartmouse.mlpstyle")


# In[2]:


def simulate(V, J, C, K, R, L, theta_dot=0, i=0):
    T = 5
    dt = 0.01
    ts = np.arange(0, T, dt)
    theta_dots = np.zeros(ts.shape[0])
    currents = np.zeros(ts.shape[0])
    for idx, t in enumerate(ts):
        A = np.array([[-C/J,K/J],[-K/L,-R/L]])
        B = np.array([0, 1/L])
        x = np.array([theta_dot, i])
        u = V(t, i, theta_dot)
        x_dot = A@x+B*u
        theta_dot += (x_dot[0] * dt)
        i += (x_dot[1] * dt)
        theta_dots[idx] = theta_dot
        currents[idx] = i
    
    return ts, theta_dots, currents

def const_V(v):
    def _v(t, i, theta_dot):
        return v
    return _v

ts, theta_dots, _ = simulate(const_V(1), J=0.01, C=0.1, K=0.01, R=1, L=0.5)
plt.figure()
plt.plot(ts, theta_dots)
plt.title("Motor response to 1v input")
plt.ylabel("Speed (rad/s)")
plt.xlabel("Time (seconds)")

ts, theta_dots, _ = simulate(const_V(5), J=0.01, C=0.1, K=0.01, R=1, L=0.5)
plt.figure()
plt.plot(ts, theta_dots)
plt.title("Motor response to 5v input")
plt.ylabel("Speed (rad/s)")
plt.xlabel("Time (seconds)")

ts, theta_dots, _ = simulate(const_V(5), J=0.05, C=0.1, K=0.01, R=1, L=0.5)
plt.figure()
plt.plot(ts, theta_dots)
plt.title("Heavier motor response to 5v input")
plt.ylabel("Speed (rad/s)")
plt.xlabel("Time (seconds)")

ts, theta_dots, _ = simulate(const_V(5), J=0.05, C=0.1, K=0.01, R=1, L=0.5, theta_dot=1)
plt.figure()
plt.plot(ts, theta_dots)
plt.title("Already spinning motor response to 5v input")
plt.ylabel("Speed (rad/s)")
plt.xlabel("Time (seconds)")

plt.show()


# In[3]:


ts, theta_dots, currents = simulate(const_V(5), J=0.000658, C=0.0000024571, K=0.0787, R=5, L=0.58)

print('max speed', theta_dots[-1]*0.0145)

plt.figure(figsize=(11,5))
plt.subplot(121)
plt.plot(ts, theta_dots * 0.0145)
plt.title("Speed of realistic motor")
plt.ylabel("Speed (meter/s)")
plt.xlabel("Time (seconds)")

plt.subplot(122)
plt.plot(ts, currents)
plt.title("Current of realistic motor")
plt.ylabel("Current (amps)")
plt.xlabel("Time (seconds)")
plt.show()


# In[4]:


ts, theta_dots, currents = simulate(const_V(0), J=0.000658, C=0.0000024571,
                                    K=0.0787, R=5, L=0.58, theta_dot=63, i=0.00198)

plt.figure(figsize=(11,5))
plt.plot(ts, theta_dots * 0.0145)
plt.title("Speed of realistic motor")
plt.ylabel("Speed (meter/s)")
plt.xlabel("Time (seconds)")

# integrate theta_dots to get theta
x = 0
xs = []
for theta_dot in theta_dots:
    x = x + theta_dot*0.01*0.0145
    xs.append(x)

plt.figure(figsize=(10,10))
plt.plot(ts, xs)
plt.ylabel("Position (m)")
plt.xlabel("Time (seconds))")
plt.show()


# we can use the slope of this line as our feed forward slope
# We just need to convert from $\frac{v}{m*s^{-1}}$ to $\frac{f}{rad*s^{-1}}$
# 
# $$ \frac{\text{volts}}{m*s^{-1}}*\frac{255\text{ abstract force}}{5\text{ volts}}*\frac{0.0145\text{ meters}}{\text{radians}} $$
# 
# The invert this value.

# ## Wheel inertia calculation
# 
# $$ I_z = \frac{mr^2}{2} = \frac{0.0122*0.014^2}{2} = 0.0000011956 $$
# 
# \*source for mass & radius: https://www.pololu.com/product/1127

# # Analyzing and Tuning Our Model
# 
# We can use experimental data from our motors to determine how accurate our model is, and to automatically tune the parameters to make it more accurate. This is important because it means work done in simulation will better translate to the real world.

# In[33]:


# load our data
data = np.genfromtxt("./impulse_response_data/with_load_left_1.csv", skip_header=True, delimiter=',')
N = data.shape[0]
t = data[:,0]
speeds = np.zeros((N, 1))
for i in range(1, N):
    speeds[i] = (data[i,1] - data[i-1,1]) / (data[i,0] - data[i-1,0])


# In[34]:


plt.figure(figsize=(15,15))
plt.scatter(t, speeds, s=1)
plt.title("Encoder Speed")
plt.xlabel("Time (micros)")
plt.ylabel("Ticks")
plt.show()

