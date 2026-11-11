import numpy as np
from src.autonomy.utils import *

class USV_Control:
    
    def controller(self, v_real:float, psi_real_rad:float, v_desired:float, psi_desired_rad:float)->None:
        """
        For each physic_step, this function computes a command and comunnicates it to the motors using the input.
        """

        delta_v =  v_desired - v_real
        delta_psi_rad =  sawtooth(psi_desired_rad - psi_real_rad)


        #TODO! Faire une vraie etude pour trouver les bons gains, le tatonnement ne suffit pas!!!
        Kp_v = 5
        Kp_psi  = 250

        U_sum_feedforward = 255 * (v_desired/self.max_speed)

        U_sum_feedback = Kp_v * (delta_v / self.max_speed) # FeedBack: closing the loop using current USV state (Proportionnal)
        U_diff_feedback = Kp_psi * (delta_psi_rad / np.pi) # FeedBack: closing the loop using current USV state (Proportionnal)

        Ul  = U_sum_feedforward + U_sum_feedback - U_diff_feedback
        Ur  = U_sum_feedforward + U_sum_feedback + U_diff_feedback
        if Ul<0 or Ur<0:
            print("Warning: negative motor command, the USV will not move. Check the controller gains and the max_speed parameter.")
        return Ul, Ur