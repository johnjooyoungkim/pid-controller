# PID (Proportional-Integral-Derivative) Controller Class

import numpy as np

class PID: 
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain
        self.integral = 0.0
        self.previous_error = 0.0
        self.history = []

        

    def compute(self, command, output, dt):
        error = command - output
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        self.previous_error = error
 
        P_term = self.Kp * error
        I_term = self.Ki * self.integral
        D_term = self.Kd * derivative
        u = P_term + I_term + D_term

        self.history.append([P_term, I_term, D_term, u])

        return u

    # params: none
    # return: numpy array [P_term, I_term, D_term, u] 
    def return_history(self):
        return np.array(self.history)

    def reset_history(self):
        self.history = []
