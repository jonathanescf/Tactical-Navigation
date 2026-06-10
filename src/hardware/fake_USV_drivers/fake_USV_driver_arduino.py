from src.fleet.USV import USV

class fake_USV_ardu():
    def __init__(self, USV:USV):
        """
        This class simulates the arduino motor driver. It takes the motor commands that would be sent to the arduino and updates the USV state accordingly. It serves as an intermediate between the navigation algorithms and the USV physics.
        """
        self.USV = USV
        self.cmd_left = 0
        self.cmd_right = 0
        print('Init Arduino ...')

    def bound_cmd (self,cmd0:float)->float:
        """
        Asserts the motor command is between -255 and 255. This is the range of the motor command that can be sent to the arduino.
        """
        cmd = cmd0
        if cmd > 255:
            cmd = 255
        if cmd < -255:
            cmd = -255
        return cmd
    
    def send_arduino_cmd_motor(self,cmdl0:float,cmdr0:float)->None:    
        """
        sends the motor commands to the fake driver class to then simulate the USV movements.
        """
        self.cmd_left = self.bound_cmd(cmdl0)
        self.cmd_right = self.bound_cmd(cmdr0)
        self.USV.Ul = self.cmd_left
        self.USV.Ur = self.cmd_right

    def get_arduino_cmd_motor(self)->tuple[float,float]:
        """
        returns the the motor commands that would be sent to the arduino
        """
        return self.cmd_left, self.cmd_right
        


        


    

    
        


    