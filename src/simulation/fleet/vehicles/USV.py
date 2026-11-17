import numpy as np
import threading 

from src.simulation.fleet.hardware.fake_USV_motor_driver import fake_USV_ardu
from src.simulation.fleet.hardware.fake_GPS_driver import fake_USV_gps
from src.simulation.fleet.hardware.fake_IMU_driver import fake_USV_imu

from src.autonomy.brain import USV_brain


class USV_body:
    def __init__(self, x0:float = None, y0:float = None, psi_rad0:float = 0, 
                 terrain=None):

        self.usv_lock = threading.Lock()

        self.terrain = terrain

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

    def get_space_state(self):
        with self.usv_lock:
            return self._x, self._y, self._psi_rad, self._Ul, self._Ur
            
            