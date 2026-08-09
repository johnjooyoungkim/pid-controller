import numpy as np

class Plant:
    def __init__(self, mass, friction, initial_state):
        self.mass = mass
        self.friction = friction # friction coefficient
        self.state = initial_state  # [position, velocity]

    def step(self, control, dt):
        # update the state of the plant based on control input and time step
        self.state[1] = self.state[1] + (control - self.friction * self.state[1]) / self.mass * dt
        self.state[0] = self.state[0] + self.state[1] * dt  # update position 
        