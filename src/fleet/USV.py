import numpy as np
import threading 

from src.hardware.fake_USV_drivers.fake_USV_driver_arduino import fake_USV_ardu
from src.hardware.fake_USV_drivers.fake_USV_driver_gps import fake_USV_gps
from src.hardware.fake_USV_drivers.fake_USV_driver_imu import fake_USV_imu

from src.user_scripts.src.USV__master import USV_master


class USV:
    def __init__(self, id:int, x0:float = 0, y0:float = 0, psi_rad0:float = 0):

        self.usv_lock = threading.Lock()

        self._id = id
        self._x = x0
        self._y = y0
        self._psi_rad = psi_rad0
        self._Ul = 0
        self._Ur = 0

        self.fake_ardu = fake_USV_ardu(USV = self)
        self.fake_gps  = fake_USV_gps(USV = self)
        self.fake_imu  = fake_USV_imu(USV = self)
        
        self.do  = USV_master(ardu = self.fake_ardu,gps = self.fake_gps,imu = self.fake_imu)

    @property
    def x(self):
        with self.usv_lock:
            return self._x

    @property
    def y(self):
        with self.usv_lock:
            return self._y

    @property
    def psi_rad(self):
        with self.usv_lock:
            return self._psi_rad

    @property
    def Ul(self):
        with self.usv_lock:
            return self._Ul
        
    @property
    def Ur(self):
        with self.usv_lock:
            return self._Ur

    @x.setter
    def x(self, value):
        with self.usv_lock:
            self._x = value

    @y.setter
    def y(self, value):
        with self.usv_lock:
            self._y = value

    @psi_rad.setter
    def psi_rad(self, value):
        with self.usv_lock:
            self._psi_rad = value

    @Ul.setter
    def Ul(self, value):
        with self.usv_lock:
            self._Ul = value
    
    @Ur.setter
    def Ur(self, value):
        with self.usv_lock:
            self._Ur = value

    def physic_step(self, dt:float):
        """
        Used to update the USV state using physics equations.
        K and L are arbitrary, further research should be conducted to make the physics more realistic
        """
        print("physic_step acquiring the usv_lock...")

        with self.usv_lock:
            print("physic_step has the usv_lock...")
            # modèle idéal : vitesse instantanée
            K = 0.05
            L = 0.25
            v     = 2 * K * (self._Ul + self._Ur) / 2
            omega = K * (self._Ur - self._Ul) / (2 * L)
            self._x   += v * np.cos(self._psi_rad) * dt
            self._y   += v * np.sin(self._psi_rad) * dt
            self._psi_rad  = (self._psi_rad + omega * dt) % (2 * np.pi)
        print("physic_step releases the usv_lock...")
            