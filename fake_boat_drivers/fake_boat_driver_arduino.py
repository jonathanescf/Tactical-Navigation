



class fake_boat_ardu():
    def __init__(self, ddboat_state):
        """
        This class simulates the arduino motor driver. It takes the motor commands that would be sent to the arduino and updates the boat state accordingly. It serves as an intermediate between the navigation algorithms and the boat physics.
        """
        self.boat = ddboat_state
        self.cmd_left = 0
        self.cmd_right = 0
        print('Init Arduino ...')

    def bound_cmd (self,cmd0):
        """
        Asserts the motor command is between -255 and 255. This is the range of the motor command that can be sent to the arduino.
        """
        cmd = cmd0
        if cmd > 255:
            cmd = 255
        if cmd < -255:
            cmd = -255
        return cmd
    
    def send_arduino_cmd_motor(self,cmdl0,cmdr0):    
        """
        sends the motor commands to the fake driver class to then simulate the boat movements.
        """
        self.cmd_left = self.bound_cmd(cmdl0)
        self.cmd_right = self.bound_cmd(cmdr0)
        self.boat.Ul = self.cmd_left
        self.boat.Ur = self.cmd_right

    def get_arduino_cmd_motor(self):
        """
        returns the the motor commands that would be sent to the arduino
        """
        return self.cmd_left, self.cmd_right
        


        


    

    
        


    