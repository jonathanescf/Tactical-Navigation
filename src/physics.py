import numpy as np



class ddboat_state():
    def __init__(self,id, x0 = 0, y0 = 0, psi_rad0 = 0):
        self.id = id
        self.x = x0
        self.y = y0
        self.psi_rad = psi_rad0
        self.Ul = 0
        self.Ur = 0
      
    def step(self, dt):
        """
        Used to update the boqt state using physics equations.
        K and L are arbitrary, further research should be conducted to make the physics more realistic
        """
        # modèle idéal : vitesse instantanée
        K = 0.05
        L = 0.25
        v     = K * (self.Ul + self.Ur) / 2
        omega = K * (self.Ur - self.Ul) / (2 * L)
        self.x   += v * np.cos(self.psi_rad) * dt
        self.y   += v * np.sin(self.psi_rad) * dt
        self.psi_rad  = (self.psi_rad + omega * dt) % (2 * np.pi)
        
        