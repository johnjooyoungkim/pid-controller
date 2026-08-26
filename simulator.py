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
                state_index, # index of the controlled state
                noise_std):

        # record of state change
        history = np.zeros((n,len(self.plant.state)))


        # step through 
        #for i in range(n):
        #    noisy_state = plant.state[state_index] + np.random.normal(loc=0, scale=noise_std) # add sensor noise
        #    plant.step(pid.compute(command=command, output=noisy_state, dt=dt, u_min=u_min, u_max=u_max), dt) # compute control input based on position 100 
        #    history[i] = plant_drone.state

        #for i in range(int(n/2), n):
        #    noisy_state = plant_drone.state[0] + np.random.normal(loc=0, scale=noise_std) # add sensor noise

        #    plant_drone.step(ctrl_PID.compute(command=cmd2, output=noisy_state, dt=dt, u_min=u_min, u_max=u_max), dt) # compute control input based on position 100 
        #    history[i] = plant_drone.state



        return history