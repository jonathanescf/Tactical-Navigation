import numpy as np
from src.hardware.fake_USV_drivers.fake_USV_driver_gps import LAT0, LON0, METER_PER_DEG_LAT, METER_PER_DEG_LON

def sawtooth(x:float)->float:
    """
    Used to correctly handle angle modulos
    """
    return (x+np.pi)%(2*np.pi)-np.pi

def nmea_to_decimal(nmea_val:float)->float:
    """
    Function to convert GNSS data in missions
    """
    d = int(nmea_val / 100)        # partie degrés
    m = nmea_val - d * 100         # partie minutes
    return d + m / 60.0            # degrés décimaux

def latlon_to_xy(lat:float, lon:float)->tuple[float]:
    """"
    Conversion function to convert lat lon coordinates. Should be improved using existing libraries like pyproj.
    """
    x = (lon - LON0) * METER_PER_DEG_LON
    y = (lat - LAT0) * METER_PER_DEG_LAT
    return x, y

def vector_to_heading(vector:np.array)->float:
    """
    Function to convert a vector to a heading in radians.
    """
    return np.arctan2(vector[1], vector[0])