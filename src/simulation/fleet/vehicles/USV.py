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
        self.max_speed = 25.8
        self.terrain = terrain


        self.init_USV_state(x0, y0, psi_rad0)


    def init_USV_state(self, x0, y0, psi_rad0):
    
        self.Ul, self.Ur = 0, 0
        self.Tl, self. Tr = 0, 0

        # absolute position in world frame in ENU
        if x0 is not None:
            self.eta = np.array([x0, y0, psi_rad0], dtype=float)
        else:
            self.eta = np.array([self.terrain.spawn[0],
                                self.terrain.spawn[1],
                                psi_rad0], dtype=float)

        # In the USV body frame
        self.nu = np.array([0, 0, 0], dtype=float) # [u, v, r]

        
    @property
    def u(self):            # u is SURGE: longitudinal speed
        return self.nu[0] 

    @property
    def v(self):            # v is SWAY: lateral speed
        return self.nu[1] 

    @property
    def r(self):            # r is YAW: rotaionnal speed around vertical axis
        return self.nu[2] 

    @property
    def x(self):            
        return self.eta[0] 

    @property
    def y(self):            
        return self.eta[1] 

    @property
    def psi_rad(self):   
        return self.eta[2] 

    def get_space_state(self):
        with self.usv_lock:
            return self.x, self.y, self.psi_rad, self.u, self.v, self.r, self.Ul, self.Ur, self.Tl, self.Tr
            
            