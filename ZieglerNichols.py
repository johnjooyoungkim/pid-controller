# find_ultimate_gain(), ziegler_nichols_pid() 

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

from pid import PID
from plant import Plant_Drone, Plant_Car
from simulator import Simulator

def ziegler_nichols_pid(Ku, Tu):
    return dict(Kp=0.6*Ku, Ki=2*0.6*Ku/Tu, Kd=0.6*Ku*Tu/8)

def find_ultimate_gain(
    plant, 
    command,
    state_index=0,
    tolerance=0.05, # tolerance for critical point (percentage)
    kp_start=0.0, 
    kp_step=0.01, 
    dt=0.01, 
    n=10,
    kp_max=100):

    kp = kp_start
    simulator = Simulator(plant, dt, n)
    initial_state = plant.state.copy()
    while kp < kp_max:
        plant.state = initial_state.copy()
        pid = PID(Kp=kp, Ki=0, Kd=0)
        history = simulator.simulate(pid=pid, 
                command=command,
                state_index=state_index, # index of the controlled state
                noise_std=0,
                u_min=None,
                u_max=None)
        history = history[:, 0]

        if is_critical_point(history, tolerance):
            peaks,_ = signal.find_peaks(history)
            Tu = measure_period(peaks=peaks,dt=dt)
            Ku = kp
            # return Ku, Tu
            return Ku, Tu
        kp += kp_step
    raise RuntimeError(f"No sustained oscillation found up to kp={kp_max}")
    
def is_critical_point(history, # 1d numpy array with the history of the controlling variable
                    tolerance=0.05): # percentage of tolerance

    peaks,_ = signal.find_peaks(history) # peaks: (ndarray) peak amplitudes of history
    if len(peaks) < 4:
        return False
    amplitudes = [abs(history[p]) for p in peaks[-4:]]
    return (max(amplitudes) - min(amplitudes)) / max(amplitudes) < tolerance

def measure_period(peaks, dt, n_last=4):
    peaks = peaks[-n_last:]  # last few peak indices
    intervals = np.diff(peaks) * dt  # convert index gaps to time
    return np.mean(intervals)