# PID (Proportional-Integral-Derivative) Controller Class

import numpy as np

class PID: 
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_output = None
        self.history = []
        self.error = []
        

    def compute(self, command, output, dt, u_min=None, u_max=None):
        error = command - output

        # first call -> no spike on D-term
        if self.previous_output is None:
            self.previous_output = output

        # Derivative
        derivative = -(output - self.previous_output) / dt # derivative-on-output
        self.previous_output = output

        temp_integral = self.integral + error * dt

        P_term = self.Kp * error
        I_term = self.Ki * temp_integral
        D_term = self.Kd * derivative
        
        u_unclamped = P_term + I_term + D_term

        if u_min is not None and u_max is not None:

            u_clamped = np.clip(u_unclamped, u_min, u_max)
            
            if (u_clamped != u_unclamped) and u_unclamped * error > 0: # condition for clamp
                u = u_clamped
            else:
                self.integral = temp_integral # integrator remains on
                u = u_unclamped

        else:
            self.integral = temp_integral
            u = u_unclamped
        
        self.previous_error = error
        self.history.append([P_term, I_term, D_term, u])
        self.error.append(error)
        return u

    # params: none
    # return: numpy array: history of [P_term, I_term, D_term, u] terms
    def return_history(self):
        return np.array(self.history)

    # params: none
    # return: numpy array: history of error values
    def return_error(self):
        return np.array(self.error)

    def reset(self):
        self.history = []
        self.error = []