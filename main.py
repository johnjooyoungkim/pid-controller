import numpy as np
import matplotlib.pyplot as plt

from plant import Plant_Car, Plant_Drone
from pid import PID
from simulator import Simulator
from ZieglerNichols import ziegler_nichols_pid, find_ultimate_gain


dt = 0.05 # time step
n = 2000  # number of steps

# saturation
u_max = 30
u_min = -30

plant = Plant_Car(mass=1.0, friction=0.05, initial_state=[0.0, 1.0])
plant_drone = Plant_Drone(mass=1.0, friction=0.05, initial_state=[0.0, 0.0], saturation=u_max)

# setpoint
cmd = 150

# noise value (standard deviation)
noise_std = 0.1

# pid controller init
pid = PID(Kp=0.50, Ki=0.02, Kd=0.5)


#Z-N
Ku, Tu = find_ultimate_gain(plant=plant,
                            command=cmd,
                            tolerance=0.05, # tolerance for critical point (percentage)
                            kp_start=0.0, 
                            kp_step=0.01, 
                            dt=dt, 
                            n=n,
                            kp_max=100)
zn = ziegler_nichols_pid(Ku=Ku, Tu=Tu)
pid_ZN = PID(Kp=zn["Kp"], Ki=zn["Ki"], Kd=zn["Kd"])


# SIMULATION
simulator_drone = Simulator(plant=plant_drone, dt=dt, n=n)

history = simulator_drone.simulate(pid=pid_ZN, 
                command=cmd,
                state_index=0, # index of the controlled state
                noise_std=noise_std,
                u_min=u_min,
                u_max=u_max)

# plot dynamics
simulator_drone.plot(pid=pid_ZN, history=history, dt=dt,n=n)
