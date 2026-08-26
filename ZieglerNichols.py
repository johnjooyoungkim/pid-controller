# find_ultimate_gain(), ziegler_nichols_pid() 

import numpy as np
import matplotlib.pyplot as plt

from pid import PID
from plant import Plant_Drone, Plant_Car
from simulator import Simulator

def compute_gains(Ku, Tu):

def find_ultimate_gain(
    plant, 
    tolerance=0.05, # tolerance for critical point (percentage)
    kp_start=0.0, 
    kp_step=0.01, 
    dt=0.01, 
    n=10):

    kp = kp_start
    simulator = Simulator(plant, dt, n)

    # 
    while True:
        pid = PID(Kp=Kp, Ki=0, Kd=0)
        history = simulator.simulate(pid)

        if is_critical_point(history, tolerance):
            # measure Tu

            # return Kp, Tu

        kp += kp_step

        
def is_critical_point(history, # 1d numpy array with the history of the controlling variable
                    tolerance=0.05) # percentage of tolerance
    peaks = find_peaks(history) # peaks: numpy array with peak amplitudes of history
    if len(peaks) < 4:
        return False
    amplitudes = [abs(p) for p in peaks[-4:]]
    return (max(amplitudes) - min(amplitudes)) / max(amplitudes) < tolerance