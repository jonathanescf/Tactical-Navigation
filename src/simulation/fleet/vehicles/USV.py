import numpy as np
import threading 

from src.simulation.fleet.hardware.fake_USV_motor_driver import fake_USV_ardu
from src.simulation.fleet.hardware.fake_GPS_driver import fake_USV_gps
from src.simulation.fleet.hardware.fake_IMU_driver import fake_USV_imu

from src.autonomy.brain import USV_brain


class USV:
    def __init__(self, id:str, x0:float = None, y0:float = None, psi_rad0:float = 0, 
                 terrain=None):

        self.usv_lock = threading.Lock()

        self.terrain = terrain

        self._id = id
        self._type = "USV"
        self.max_speed = 6 # m/s
        self.lenght = 2
        if x0 is not None:
            self._x = x0
            self._y = y0
            self._psi_rad = psi_rad0
        else:
            self._x = self.terrain.spawn[0]
            self._y = self.terrain.spawn[1]
            self._psi_rad = psi_rad0
        self._Ul = 0
        self._Ur = 0

        self.fake_ardu = fake_USV_ardu(USV = self)
        self.fake_gps  = fake_USV_gps(USV = self)
        self.fake_imu  = fake_USV_imu(USV = self)
        
        self.do  = USV_brain(ardu = self.fake_ardu,
                             gps = self.fake_gps, 
                             imu = self.fake_imu,
                             max_speed = self.max_speed)


    @property
    def id(self):
        with self.usv_lock:
            return self._id

    @property
    def type(self):
        with self.usv_lock:
            return self._type

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

        with self.usv_lock:
            # modèle idéal : vitesse instantanée
            K = self.max_speed / 255
            L = 1
            v     =  K * (self._Ul + self._Ur) / 2
            omega = K * (self._Ur - self._Ul) / (2 * L)

            x_new = self._x + v * np.cos(self._psi_rad) * dt
            y_new = self._y + v * np.sin(self._psi_rad) * dt

            if self.terrain.distance(x_new, y_new) < 0:
                print("WARNING: COLLISION WITH AN OBSTACLE")
                self._psi_rad  = (self._psi_rad + omega * dt + np.pi) % (2 * np.pi) 
                self._x = x_new
                self._y = y_new

            else:
                self._x = x_new
                self._y = y_new
                self._psi_rad  = (self._psi_rad + omega * dt) % (2 * np.pi)
            
            