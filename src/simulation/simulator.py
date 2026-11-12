from time import time




class simulator:
    def __init__(self, terrain, bodies, dt):
        self.terrain = terrain
        self.bodies = bodies
        self.t = 0.0
        self.dt = dt

    def global_physic_step(self):
        for body in self.bodies.values():
            body.physic_step(self.dt)
        self.t += self.dt