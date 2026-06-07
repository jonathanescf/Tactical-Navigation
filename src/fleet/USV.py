import numpy as np
from src.hardware.fake_USV_drivers.fake_USV_driver_arduino import fake_USV_ardu
from src.hardware.fake_USV_drivers.fake_USV_driver_gps import fake_USV_gps
from src.hardware.fake_USV_drivers.fake_USV_driver_imu import fake_USV_imu

from src.user_scripts.src.USV__master import USV_master


class USV:
    def __init__(self, id, x0 = 0, y0 = 0, psi_rad0 = 0):
        self.id = id
        self.x = x0
        self.y = y0
        self.psi_rad = psi_rad0
        self.Ul = 0
        self.Ur = 0


        self.fake_ardu = fake_USV_ardu(USV = self)
        self.fake_gps  = fake_USV_gps(USV = self)
        self.fake_imu  = fake_USV_imu(USV = self)
        
        self.do  = USV_master(ardu = self.fake_ardu,gps = self.fake_gps,imu = self.fake_imu)
        

    
    def step(self, dt):
        """
        Used to update the USV state using physics equations.
        K and L are arbitrary, further research should be conducted to make the physics more realistic
        """
        # modèle idéal : vitesse instantanée
        K = 0.05
        L = 0.25
        v     = 2 * K * (self.Ul + self.Ur) / 2
        omega = K * (self.Ur - self.Ul) / (2 * L)
        self.x   += v * np.cos(self.psi_rad) * dt
        self.y   += v * np.sin(self.psi_rad) * dt
        self.psi_rad  = (self.psi_rad + omega * dt) % (2 * np.pi)