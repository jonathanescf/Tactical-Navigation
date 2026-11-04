import numpy as np
from src.autonomy.utils import *

class USV_Control:
    
    def controller(self, v_real:float, psi_real_rad:float, v_desired:float, psi_desired_rad:float)->None:
        """
        For each physic_step, this function computes a command and comunnicates it to the motors using the input.
        """

        delta_psi_rad =  -sawtooth(psi_real_rad - psi_desired_rad)

        Kp  = 200          
        Ul  = v_desired - Kp * (delta_psi_rad / np.pi)
        Ur  = v_desired + Kp * (delta_psi_rad / np.pi)

        return Ul, Ur