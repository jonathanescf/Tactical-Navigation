import numpy as np
from src.user_scripts.utilities.utils import *


class USV_sensors:
    def current_heading(self)->float:
        """
        Function to get the heading of the USV using the magnetometer of the IMU. The unit is in radians.
        """
        mag = self.imu.read_mag_raw()
        return -np.arctan2(mag[1], mag[0])
    
    #TODO: Implémentez un filtre de kalman sur l'imu et le gps pour estimer la vitesse du bateau
    def current_speed(self)->float:
        """
        The speed is taken from the motor command, K is given to have an exact estimate. This is ideal and should be changed.
        """
        K = 0.05
        Ul, Ur = self.ardu.get_arduino_cmd_motor()
        v     = K * (Ul + Ur) / 2
        return v
        
    def current_lat_lon_position(self)->tuple[float,float]:
        """
        Function to get the current latitude and longitude of the USV using the GPS. 
        The unit is in decimal degrees. 
        """
        msg, val = self.gps.read_gll_non_busv_locking()
        if msg:
            lat = nmea_to_decimal(val[0])   
            lon = nmea_to_decimal(val[2])
            if val[1] == 'S': lat = -lat
            if val[3] == 'W': lon = -lon
            return lat, lon
        return None, None
        
    def current_x_y_position(self)->tuple[float,float]:
        """
        Conversion function to getx,y coordinates from latitude and longitude. The unit is in meters. 
        The origin (0,0) is found in he latlon_to_xy function in utils.py.
        """
        print("getting lat lon USV position...")
        lat, lon = self.current_lat_lon_position()
        print("getting x,y fron lat lon...")
        if lat is not None and lon is not None:
            x, y = latlon_to_xy(lat, lon)
        else:
            x, y = None, None
        return x, y