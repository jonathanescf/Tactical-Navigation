import threading
import time

from src.map.mapbuilder import mapbuilder
from src.graphics.graphical_interface import graphical_interface
from src.fleet.fleet_manager import fleet_manager

class software:

    def __init__(self, parameters):

        self.software_running = True
        self.parameters = parameters
        # Create instances of the main classes
        self.fleet_manager = fleet_manager(self.parameters)
        self.map_builder = mapbuilder(self.parameters)
        self.graphical_interface = graphical_interface(self.parameters, fleet_manager = self.fleet_manager, map_builder = self.map_builder)
        
        # Start everything in separate threads
        self.thread_fleet_dict = {}

        self.thread_IHM = threading.Thread(target=self.launch, daemon=True)
        self.thread_IHM.start()

        self.thread_physics_loop = threading.Thread(target=self.physics_loop, daemon=True)
        self.thread_physics_loop.start()

        # dans software.__init__, avant graphical_interface.run()
        # def monitor():
        #     while True:
        #         print("─── threads actifs ───")
        #         for t in threading.enumerate():
        #             print(f"  {t.name} — alive: {t.is_alive()}")
        #         time.sleep(1)

        # threading.Thread(target=monitor, daemon=True).start()

        self.graphical_interface.run()
        
    def physics_loop(self):
        while self.software_running:
            print("physics tick")
            for USV in self.fleet_manager.USV.values():
                USV.physic_step(self.parameters["dt"])
            time.sleep(self.parameters["dt"])

    def launch(self):
        # input("Appuie sur Entrée pour tester les moteurs...")

        for i in range(1, self.fleet_manager.nb_USVs + 1):
            self.thread_fleet_dict[i] = threading.Thread(target=self.fleet_manager.USV[i].do.go_to_waypoints, daemon=True)
            self.thread_fleet_dict[i].start()
            time.sleep(6.5)        
    
    #TODO: should stop every thread with a stop event.

        
