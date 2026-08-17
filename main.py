import numpy as np
import matplotlib.pyplot as plt

from plant import Plant_Car, Plant_Drone
from pid import PID

dt = 0.1 # time step
n = 3000  # number of steps

# saturation
u_max = 14
u_min = -14

plant = Plant_Car(mass=1.0, friction=0.05, initial_state=[0.0, 1.0])
plant_drone = Plant_Drone(mass=1.0, friction=0.05, initial_state=[0.0, 0.0], saturation=u_max)

# record of state change
history = np.zeros((n,2))

# setpoint
cmd = 10

# noise value (standard deviation)
noise_std = 0

# pid controller init
ctrl_PID = PID(Kp=0.51, Ki=0.02, Kd=0.5)


# step through 
for i in range(n):
    noisy_state = plant_drone.state[0] + np.random.normal(loc=0, scale=noise_std) # add sensor noise

    plant_drone.step(ctrl_PID.compute(command=cmd, output=noisy_state, dt=dt, u_min=u_min, u_max=u_max), dt) # compute control input based on position 100 
    history[i] = plant_drone.state

# plot dynamics
fig, (ax_x, ax_v, ax_e, ax_pid) = plt.subplots(4,1, figsize=(6,11))
ax_x.plot(np.linspace(0,dt*n, n), history[:,0])
ax_x.set_ylabel("position")
ax_v.plot(np.linspace(0,dt*n, n), history[:,1])
ax_v.set_ylabel("velocity")

# plot PID values + error
pid_history = ctrl_PID.return_history()
error_history = ctrl_PID.return_error()

ax_e.plot(np.linspace(0, dt*n, n), error_history)
ax_e.set_ylabel("error")
ax_e.axhline(0, color='black', linestyle='-')
ax_e.fill_between(np.linspace(0, dt*n, n), error_history, color="skyblue", alpha=0.4)

ax_pid.plot(np.linspace(0,dt*n, n), pid_history[:,0],  label="P term")
ax_pid.plot(np.linspace(0,dt*n, n), pid_history[:,1], label="I term")
ax_pid.plot(np.linspace(0,dt*n, n), pid_history[:,2], label="D term")
ax_pid.plot(np.linspace(0,dt*n, n), pid_history[:,3], label="total control", linestyle="--")
ax_pid.legend()
ax_pid.axhline(0, color='black', linestyle='-')
ax_pid.set_xlabel("timestep")
ax_pid.set_ylabel("control contribution")

plt.suptitle("PID control on drone movement")
plt.savefig('output.png')

