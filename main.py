import numpy as np
import matplotlib.pyplot as plt

from plant import Plant
from pid import PID

dt = 0.1 # time step
n = 1000  # number of steps
plant = Plant(mass=1.0, friction=0.05, initial_state=[0.0, 1.0])

# record of state change
history = np.zeros((n,2))

# setpoint
cmd = 100 

# noise value (standard deviation)
noise_std = 1

# pid controller init
ctrl_PID = PID(Kp=0.5, Ki=0.01, Kd=0.2)

# step through 
for i in range(n):
    noisy_state = plant.state[0] + np.random.normal(loc=0, scale=noise_std) # add sensor noise

    plant.step(ctrl_PID.compute(command=cmd, output=noisy_state, dt=dt), dt) # compute control input based on position 100 
    history[i] = plant.state

# plot dynamics
fig, (ax_x, ax_v, ax_pid) = plt.subplots(3,1, figsize=(6,10))
ax_x.plot(np.linspace(0,dt*n, n), history[:,0])
ax_x.set_ylabel("position")
ax_v.plot(np.linspace(0,dt*n, n), history[:,1])
ax_v.set_ylabel("velocity")

# plot PID values
pid_history = ctrl_PID.return_history()
ax_pid.plot(np.linspace(0,dt*n, n), pid_history[:,0],  label="P term")
ax_pid.plot(np.linspace(0,dt*n, n), pid_history[:,1], label="I term")
ax_pid.plot(np.linspace(0,dt*n, n), pid_history[:,2], label="D term")
# ax.plot(np.linspace(0,dt*n, n), pid_history[:,3], label="total control", linestyle="--")
ax_pid.legend()
ax_pid.set_xlabel("timestep")
ax_pid.set_ylabel("control contribution")
plt.savefig('output.png')