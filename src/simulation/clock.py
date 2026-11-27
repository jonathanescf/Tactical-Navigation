



class clock:
    def __init__(self, dt):
        self.tick = 0
        self.t = 0.0
        self.dt = dt

    def advance(self):
        self.tick+=1
        self.t = self.tick * self.dt


        
