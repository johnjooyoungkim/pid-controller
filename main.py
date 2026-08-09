import numpy as np
import matplotlib.pyplot as plt

from plant import Plant
from pid import PID

dt = 1 # time step
n = 100  # number of steps
plant = Plant(mass=1.0, friction=0.05, initial_state=[0.0, 1.0])
history = np.zeros((n,2))

ctrl_P = PID(Kp=0.5, Ki=0.0, Kd=0.1)
ctrl_I = PID(Kp=0.0, Ki=0.01, Kd=0.0)
ctrl_D = PID(Kp=0.0, Ki=0.0, Kd=1.0)
ctrl_PID = PID(Kp=0.5, Ki=0.0, Kd=0.2)

for i in range(n):
    plant.step(ctrl_PID.compute(command=100, output=plant.state[0], dt=dt), dt) # compute control input based on position 100
    history[i] = plant.state

# plot dynamics
fig, (ax1, ax2) = plt.subplots(2,1)
ax1.plot(np.linspace(0,dt*n, n), history[:,0])
ax2.plot(np.linspace(0,dt*n, n), history[:,1])

plt.savefig('output.png')

