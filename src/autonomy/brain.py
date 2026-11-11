### In this script, I add the different methods to allow the USV to fulfill its different missions.
### For now, a simple script is efficient, a better architecture/factorization might be needed for more complex algorithms.


from src.autonomy.guidance.guidance import USV_Guidance
from src.autonomy.control import USV_Control
from src.autonomy.navigation import USV_Navigation
from src.autonomy.mission import USV_Mission



class USV_brain(USV_Mission, USV_Guidance, USV_Navigation, USV_Control):
    def __init__(self, ardu, gps, imu, max_speed): #no need to tell the type for dependancy purpose. 
                                        # USV_brain just communicates with the objects given without knowing what they are
        self.ardu = ardu
        self.gps = gps
        self.imu = imu
        self.max_speed = max_speed
        self.ongoing_target_state = None
        self.waypoints = []
        self.arrival_radius = None
        self.wp_index = 0

    def target_reached(self):
        print(self.wp_index)
        self.wp_index += 1

    def next_target(self):
        if self.wp_index >= len(self.waypoints):
            return None
        return self.waypoints[self.wp_index]

    def brain_step(self):
        nav_state = self.get_nav_state()
        self.ongoing_target_state = self.next_target()
        if self.ongoing_target_state is None:
            self.ardu.send_arduino_cmd_motor(0, 0)
            return

        Ul, Ur, TargetIntercepted = self.go_to_waypoint_step(nav_state, self.ongoing_target_state, interception_accuracy=self.arrival_radius)
        self.ardu.send_arduino_cmd_motor(Ul, Ur)

        if TargetIntercepted:
            self.target_reached()
            print("Target reached")
        return


    