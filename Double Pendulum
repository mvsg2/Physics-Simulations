# This animates an ideal system of a double pendulum swinging unimpeded in free space with no energy loss...
import numpy as np
from numpy import sin, cos
import matplotlib.pyplot as plt
import scipy.integrate as integrate
from collections import deque 
import matplotlib.animation as animation
g = 9.8                             # Acceleration due to gravity (Earth's gravity here) (in m/s^2)
L1 = 1.0                            # Length of the top pendulum (in m)
L2 = 1.0                            # Length of the bottom pendulum (in m)
m1 = 1.0                            # Mass of the first bob (in kg)
m2 = 1.0                            # Mass of the second bob (in kg)
t_stop = 60                         # Duration of the animation (in seconds) to simulate
L = L1 + L2                         # Total length of the double pendulum                              
history_len = 600                   # how many trajectory points to display
def derivs(state, t):
    dydx = np.zeros_like(state)
    dydx[0] = state[1]

    delta = state[2] - state[0]
    den1 = (m1+m2) * L1 - m2 * L1 * cos(delta) * cos(delta)
    dydx[1] = ((m2 * L1 * state[1] * state[1] * sin(delta) * cos(delta)
                + m2 * g * sin(state[2]) * cos(delta)
                + m2 * L2 * state[3] * state[3] * sin(delta)
                - (m1+m2) * g * sin(state[0]))
               / den1)
    dydx[2] = state[3]
    den2 = (L2/L1) * den1
    dydx[3] = ((- m2 * L2 * state[3] * state[3] * sin(delta) * cos(delta)
                + (m1+m2) * g * sin(state[0]) * cos(delta)
                - (m1+m2) * L1 * state[1] * state[1] * sin(delta)
                - (m1+m2) * g * sin(state[2]))
               / den2)
    return dydx

dt = 0.01
t = np.arange(0, t_stop, dt)    # Creating a time array from 0 to t_stop sampled at every 'dt' time step. 

# theta_1 and theta_2 are initial angles (in degrees) of each pendulum.
theta_1 = 120   
theta_2 = -10

# w1 and w2 are initial angular speeds (in degrees/sec) of each bob.
w1 = 0.0
w2 = 0.0 

# Now, our initial state is the collection of our initial variables defined above.
state = np.radians([theta_1, w1, theta_2, w2])
y = integrate.odeint(derivs, state, t)
x1 = L1 * sin(y[:, 0])
y1 = -L1 * cos(y[:, 0])
x2 = L2 * sin(y[:, 2]) + x1
y2 = -L2 * cos(y[:, 2]) + y1

fig = plt.figure(figsize=(5, 4))
ax = fig.add_subplot(autoscale_on = False, xlim = (-L, L), ylim = (-L, 1.))
ax.set_aspect('equal')
ax.grid()
line, = ax.plot([], [], 'o-', lw = 2)
trace, = ax.plot([], [], '.-', lw=1, ms=2)
time_template = 'time = %.1fs'
time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)
history_x, history_y = deque(maxlen=history_len), deque(maxlen=history_len)

def animate(i):
    thisx = [0, x1[i], x2[i]]
    thisy = [0, y1[i], y2[i]]

    if i == 0:
        history_x.clear()
        history_y.clear()

    history_x.appendleft(thisx[2])
    history_y.appendleft(thisy[2])
    line.set_data(thisx, thisy)
    trace.set_data(history_x, history_y)
    time_text.set_text(time_template % (i * dt))
    return line, trace, time_text

ani = animation.FuncAnimation(fig, animate, len(y), interval = dt * 1000, blit = True)
plt.show()

# Phew! That was quite a ride writing the enitre program, but I think it's all worth it in the end.
# To even have it possible that some computer program can model such an extremely chaotic dynamics to such fine extent is amazing to me in all levels. 
# This has defineitely made me appreciate what programming can accompish provided used in the right way... 
