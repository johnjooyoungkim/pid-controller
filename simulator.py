# simulator class for simulation and performance evaluation


import numpy as np
import matplotlib.pyplot as plt

from pid import PID
from plant import Plant_Drone, Plant_Car

class Simulator:

    def __init__(self, plant, dt=0.01, n=10):
        self.plant = plant
        self.dt = dt
        self.n = n

    def simulate(self, 
                pid, 
                command,
                state_index=0, # index of the controlled state
                noise_std=0,
                u_min=None,
                u_max=None):

        # record of state change
        history = np.zeros((self.n, len(self.plant.state)))

        # step through 
        for i in range(self.n):
            noisy_state = self.plant.state[state_index] + np.random.normal(loc=0, scale=noise_std) # add sensor noise
            self.plant.step(pid.compute(command=command, output=noisy_state, dt=self.dt, u_min=u_min, u_max=u_max), self.dt) # compute control input based on position 100 
            history[i] = self.plant.state

        return history

    def plot(self, pid, history, dt, n):
        fig, (ax_x, ax_v, ax_e, ax_pid) = plt.subplots(4,1, figsize=(6,11))
        ax_x.plot(np.linspace(0,dt*n, n), history[:,0])
        ax_x.set_ylabel("position")
        ax_v.plot(np.linspace(0,dt*n, n), history[:,1])
        ax_v.set_ylabel("velocity")

        # plot PID values + error
        pid_history = pid.return_history()
        error_history = pid.return_error()

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
