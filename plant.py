import numpy as np

class Plant_Car:
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

class Plant_Drone:

    def __init__(self, mass, friction, initial_state, saturation=None):
        self.mass = mass
        self.friction = friction # friction coefficient
        self.state = np.array(initial_state) # [y, v]
        self.saturation = saturation

    def step(self, control, dt):

        # actuator saturation
        if self.saturation is not None:
            control = np.clip(control, -self.saturation, self.saturation)

        # next state calculation
        accel = self.acceleration(control)
        new_velocity = self.state[1] + accel * dt # update velocity
        new_position = (self.state[0] + new_velocity) * dt # update position

        # clamping position when reaches ground
        if new_position <= 0:
            new_position = 0
            new_velocity = max(0.0, new_velocity) # allow nonzero velocity to be preserved after landing

        # update current state
        self.state = np.array([new_position, new_velocity])

    def acceleration (self, control):
        # calculate acceleration of drone and return
        return (control - self.friction * self.state[1] - self.mass * 10) / self.mass
    
