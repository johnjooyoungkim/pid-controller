# PID (Proportional-Integral-Derivative) Controller Class

class PID: 
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain
        self.integral = 0.0
        self.previous_error = 0.0

    def compute(self, command, output, dt):
        error = command - output
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        self.previous_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative