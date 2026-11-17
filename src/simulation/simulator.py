from time import time
import numpy as np



class simulator:
    def __init__(self, terrain, bodies, dt):
        self.terrain = terrain
        self.bodies = bodies
        self.t = 0.0
        self.dt = dt

    def USV_physic_step(self, body, dt:float):
        """
        Used to update the USV state using physics equations.
        K and L are arbitrary, further research should be conducted to make the physics more realistic
        """

        with body.usv_lock:
            # modèle idéal : vitesse instantanée
            K = body.max_speed / 255
            L = 1
            v     =  K * (body._Ul + body._Ur) / 2
            omega = K * (body._Ur - body._Ul) / (2 * L)

            x_new = body._x + v * np.cos(body._psi_rad) * dt
            y_new = body._y + v * np.sin(body._psi_rad) * dt

            if body.terrain.distance(x_new, y_new) < 0:
                print("WARNING: COLLISION WITH AN OBSTACLE")

            else:
                body._x = x_new
                body._y = y_new
                body._psi_rad  = (body._psi_rad + omega * dt) % (2 * np.pi)


    def global_physic_step(self):
        for body in self.bodies.values():
            self.USV_physic_step(body, self.dt)
        self.t += self.dt