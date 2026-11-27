import numpy as np
from src.autonomy.autonomy_utils import *
from collections import deque
import time


class USV_Navigation:
    def __init__(self):

        self.lat_hat = None
        self.lon_hat = None

        #eta_hat
        self.x_hat = None
        self.y_hat = None
        self.psi_hat = None

        # nu_hat
        self.u_hat = 0.0
        self.v_hat = 0.0
        self.r_hat = 0.0

        self.timestamped_position_buffer = deque(maxlen=10)
        self.t_gps_meas = None

        self.timestamped_heading_buffer = deque(maxlen=10)
        self.t_imu_meas = None

    def update_heading(self)->float:
        """
        Function to get the heading of the USV using the magnetometer of the IMU. The unit is in radians.
        """
        mag, self.t_imu_meas = self.imu.get_imu_data()
        self.psi_hat = -np.arctan2(mag[1], mag[0])
        return
    
    def update_lat_lon_position(self)->tuple[float,float]:
        """
        Function to get the current latitude and longitude of the USV using the GPS. 
        The unit is in decimal degrees. 
        """
        msg, val, self.t_gps_meas,  = self.gps.get_gps_data()
        if msg:
            lat = nmea_to_decimal(val[0])   
            lon = nmea_to_decimal(val[2])
            if val[1] == 'S': lat = -lat
            if val[3] == 'W': lon = -lon
            self.lat_hat = lat
            self.lon_hat = lon
        
    def update_x_y_position(self)->tuple[float,float]:
        """
        Conversion function to getx,y coordinates from latitude and longitude. The unit is in meters. 
        The origin (0,0) is found in he latlon_to_xy function in utils.py.
        """
        self.update_lat_lon_position()
        if self.lat_hat is not None and self.lon_hat is not None:
            self.x_hat, self.y_hat = latlon_to_xy(self.lat_hat, self.lon_hat)
        
    def update_linear_speed(self)->float:
        """
        derivating position using the position buffer
        """
        
        # update u_hat and v_hat
        if len(self.timestamped_position_buffer) >= 2:
            t1, (x1, y1) = self.timestamped_position_buffer[-2]
            t2, (x2, y2) = self.timestamped_position_buffer[-1]
            dt = t2 -t1
            if dt <= 0:
                return None
            else:
                x_dot = (x2 - x1) / dt
                y_dot = (y2 - y1) / dt

                self.u_hat = x_dot * np.cos(self.psi_hat) + y_dot * np.sin(self.psi_hat)
                self.v_hat = - x_dot * np.sin(self.psi_hat) + y_dot * np.cos(self.psi_hat)

        # updating r_hat
        if len(self.timestamped_heading_buffer) >= 2:
            t1, psi1 = self.timestamped_heading_buffer[-2]
            t2, psi2 = self.timestamped_heading_buffer[-1]
            dt = t2 -t1
            if dt <= 0:
                return None
            else:
                psi_dot = (psi2 - psi1) / dt
                self.r_hat = psi_dot

    def update_rotationnal_speed(self)->float:
        """
        derivating position using the position buffer
        """  
        # updating r_hat
        if len(self.timestamped_heading_buffer) >= 2:
            t1, psi1 = self.timestamped_heading_buffer[-2]
            t2, psi2 = self.timestamped_heading_buffer[-1]
            dt = t2 -t1
            if dt <= 0:
                return None
            else:
                psi_dot = (psi2 - psi1) / dt
                self.r_hat = psi_dot

            
    def update_nav_state(self): #each update should be called indepently at a certain freqency, then the logic should only et attributes
        previous_t_gps_meas = self.t_gps_meas
        previous_t_imu_meas = self.t_imu_meas
        self.update_x_y_position()
        self.update_heading()
        if self.t_gps_meas != previous_t_gps_meas:
            self.timestamped_position_buffer.append((self.t_gps_meas,(self.x_hat,self.y_hat)))
            self.update_linear_speed()       
        if self.t_imu_meas != previous_t_imu_meas:
            self.timestamped_heading_buffer.append((self.t_imu_meas, self.psi_hat))
            self.update_rotationnal_speed()
        return
        
    def get_nav_state(self):
        self.update_nav_state()
        return self.x_hat, self.y_hat, self.psi_hat, self.u_hat, self.v_hat, self.r_hat

    def get_heading(self):
        self.update_nav_state()
        return self.psi_hat

    def get_position(self):
        self.update_nav_state()
        return self.x_hat, self.y_hat

    def get_speed(self):
        self.update_nav_state()
        return self.u_hat, self.v_hat, self.r_hat
