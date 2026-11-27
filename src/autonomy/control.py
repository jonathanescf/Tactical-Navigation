import numpy as np
from src.autonomy.autonomy_utils import *

class USV_Control:
    
    def controller(self, u_real:float, psi_real_rad:float)->None: #obsolete
        """
        For each USV_physic_step, this function computes a command and comunnicates it to the motors using the input.
        """

        delta_u =  self.u_des - u_real
        delta_psi_rad =  sawtooth(self.psi_des - psi_real_rad)


        #TODO! Faire une vraie etude pour trouver les bons gains, le tatonnement ne suffit pas!!!
        Kp_u = 50
        Kp_psi  = 300

        U_sum_feedforward = 255 * (self.u_des/self.max_speed)

        U_sum_feedback = Kp_u * (delta_u / self.max_speed) # FeedBack: closing the loop using current USV state (Proportionnal)
        U_diff_feedback = Kp_psi * (delta_psi_rad / np.pi) # FeedBack: closing the loop using current USV state (Proportionnal)

        Ul  = U_sum_feedforward + U_sum_feedback - U_diff_feedback
        Ur  = U_sum_feedforward + U_sum_feedback + U_diff_feedback
        return Ul, Ur



