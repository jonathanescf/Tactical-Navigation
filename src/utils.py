import numpy as np
from fake_boat_drivers.fake_boat_driver_gps import LAT0, LON0, METER_PER_DEG_LAT, METER_PER_DEG_LON

def sawtooth(x):
    """
    Used to correctly handle angle modulos
    """
    return (x+np.pi)%(2*np.pi)-np.pi

def nmea_to_decimal(nmea_val):
    """
    Function to convert GNSS data in missions
    """
    d = int(nmea_val / 100)        # partie degrés
    m = nmea_val - d * 100         # partie minutes
    return d + m / 60.0            # degrés décimaux

def latlon_to_xy(lat, lon):
    """"
    Conversion function to convert lat lon coordinates. Should be improved using existing libraries like pyproj.
    """
    x = (lon - LON0) * METER_PER_DEG_LON
    y = (lat - LAT0) * METER_PER_DEG_LAT
    return x, y