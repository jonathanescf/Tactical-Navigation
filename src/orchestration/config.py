import numpy as np

# Reference point for letlon to x y conversion (the current coordinates are Paris, but they can be changed to any other location)
LAT0 = 48.8566        
LON0 =  2.3522
METER_PER_DEG_LAT = 111320.0
METER_PER_DEG_LON = 111320.0 * np.cos(np.radians(LAT0))

parameters = {
    "dt": 0.05,
    "dx": 0.1,
    "dy": 0.1,
    "nb_of_USVs": 1,
}

