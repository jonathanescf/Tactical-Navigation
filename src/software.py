import threading
import time

from src.map.mapbuilder import mapbuilder
from src.graphics.graphical_interface import graphical_interface
from src.fleet.fleet_manager import fleet_manager

class software:

    def __init__(self, parameters):

        self.software_running = True
        # Create instances of the main classes
        self.fleet_manager = fleet_manager(parameters)
        self.map_builder = mapbuilder(parameters)
        self.graphical_interface = graphical_interface(parameters, fleet_manager = self.fleet_manager, map_builder = self.map_builder)
        
        # Start everything in separate threads
        self.thread_fleet_dict = {}

        self.thread_IHM = threading.Thread(target=self.launch, daemon=True)
        self.thread_IHM.start()

        self.graphical_interface.run()
        
        

    def launch(self):
        input("Appuie sur Entrée pour tester les moteurs...")
        
        for i in range(1, self.fleet_manager.nb_boats + 1):
            self.thread_fleet_dict[i] = threading.Thread(target=self.fleet_manager.boat[i].do.go_to_waypoints, daemon=True)
            self.thread_fleet_dict[i].start()
            time.sleep(6)        
    
    #TODO: should stop every thread with a stop event.
    # def stop(self): 
    #     print("Arrêt de la simulation.")
    #     for i in range(1, self.fleet_manager.nb_boats + 1):
    #         self.fleet_manager.boat[i].do.running = False

        
