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

    def plot(self, filename, pid, history, dt, n):
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

        plt.savefig(f'{filename}.png')

    def report(self, history, command, dt):
        pos = history[:, 0]
        metrics = {
            "rise_time": rise_time(pos, command, dt),
            "overshoot_%": overshoot(pos, command),
            "settling_time": settling_time(pos, command, dt),
            "steady_state_error": steady_state_error(pos, command),
        }
        for k, v in metrics.items():
            print(f"{k}: {v:.3f}" if v is not None else f"{k}: N/A")
        return metrics

def rise_time(history, setpoint, dt, low=0.1, high=0.9):
    history = np.array(history)
    low_val, high_val = low * setpoint, high * setpoint
    try:
        t_low = np.where(history >= low_val)[0][0]
        t_high = np.where(history >= high_val)[0][0]
        return (t_high - t_low) * dt
    except IndexError:
        return None  # never reached thresholds

def overshoot(history, setpoint):
    peak = max(history) if setpoint >= 0 else min(history)
    return max(0.0, (peak - setpoint) / setpoint * 100) # percentage

def settling_time(history, setpoint, dt, tol=0.02):
    history = np.array(history)
    band = tol * abs(setpoint)
    # find last index where response is OUTSIDE the tolerance band
    outside = np.where(np.abs(history - setpoint) > band)[0]
    if len(outside) == 0:
        return 0.0
    return outside[-1] * dt  # time it last exits the band = settling time

def steady_state_error(history, setpoint, n_last=50):
    return setpoint - np.mean(history[-n_last:])

