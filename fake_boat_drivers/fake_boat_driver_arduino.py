



class fake_boat_ardu():
    def __init__(self, ddboat_state):
        self.boat = ddboat_state
        self.cmd_left = 0
        self.cmd_right = 0
        print('Init Arduino ...')

    def bound_cmd (self,cmd0):
        cmd = cmd0
        if cmd > 255:
            cmd = 255
        if cmd < -255:
            cmd = -255
        return cmd
    
    def send_arduino_cmd_motor(self,cmdl0,cmdr0):    
        self.cmd_left = self.bound_cmd(cmdl0)
        self.cmd_right = self.bound_cmd(cmdr0)
        self.boat.Ul = self.cmd_left
        self.boat.Ur = self.cmd_right

    def get_arduino_cmd_motor(self):
        return self.cmd_left, self.cmd_right
        


        


    

    
        


    