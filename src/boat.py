import numpy as np
from fake_boat_drivers.fake_boat_driver_arduino import fake_boat_ardu
from fake_boat_drivers.fake_boat_driver_gps import fake_boat_gps
from fake_boat_drivers.fake_boat_driver_imu import fake_boat_imu

from src.missions import ddboat_mission


class boat:
    def __init__(self, id, x0 = 0, y0 = 0, psi_rad0 = 0):
        self.id = id
        self.x = x0
        self.y = y0
        self.psi_rad = psi_rad0
        self.Ul = 0
        self.Ur = 0

        self.state = boat_state(x0, y0, psi_rad0)

        self.fake_ardu = fake_boat_ardu(boat = self)
        self.fake_gps  = fake_boat_gps(boat = self)
        self.fake_imu  = fake_boat_imu(boat = self)
        
        self.do  = ddboat_mission(ardu = self.fake_ardu,gps = self.fake_gps,imu = self.fake_imu)
        

    
    def step(self, dt):
        """
        Used to update the boat state using physics equations.
        K and L are arbitrary, further research should be conducted to make the physics more realistic
        """
        # modèle idéal : vitesse instantanée
        K = 0.05
        L = 0.25
        v     = K * (self.Ul + self.Ur) / 2
        omega = K * (self.Ur - self.Ul) / (2 * L)
        self.x   += v * np.cos(self.psi_rad) * dt
        self.y   += v * np.sin(self.psi_rad) * dt
        self.psi_rad  = (self.psi_rad + omega * dt) % (2 * np.pi)