import numpy as np

class Plant:
    def __init__(self, mass, friction, initial_state):
        self.mass = mass
        self.friction = friction # friction coefficient
        self.state = np.array(initial_state)  # [position, velocity]

    def step(self, control, dt):
        # update the state of the plant based on control input and time step
        accel = self.derivative(control)
        new_velocity = self.state[1] + accel * dt
        new_position = self.state[0] + new_velocity * dt  # update position
        self.state = np.array([new_position, new_velocity])
        
    def derivative(self, control):
        acceleration = (control - self.friction * self.state[1]) / self.mass
        return acceleration
    
