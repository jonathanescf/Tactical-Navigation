import numpy as np
from src.user_scripts.utilities.utils import *

class USV_control:
    def control_law(self, v_desired:float,delta_w:float)->tuple[float,float]:
        """ 
        Compute the command voltages for the left and right motors 
        based on the desired linear velocity (v_desired) and angular velocity delta_w) using a feed-forward control approach. 
        """
        Kp  = 200          
        Ul  = v_desired - Kp * (delta_w / np.pi)
        Ur  = v_desired + Kp * (delta_w / np.pi)

        return Ur, Ul
    
    def control_step(self, v_desired:float, psi_desired_rad:float)->None:
        """
        For each physic_step, this function computes a command and comunnicates it to the motors using the input.
        """

        delta_w =  -sawtooth(self.current_heading() - psi_desired_rad)
        print("going to control law...")
        Ur, Ul = self.control_law(v_desired, delta_w)
        print("going out of control law...")
        self.ardu.send_arduino_cmd_motor(Ul, Ur)