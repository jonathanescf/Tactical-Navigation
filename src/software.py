import threading
import time

from src.map.mapbuilder import mapbuilder
from src.map.mapbuilder2 import mapbuilder2
from src.graphics.graphical_interface import graphical_interface
from src.fleet.fleet_manager import fleet_manager

class software:

    def __init__(self, parameters:dict):
        print("--------------------")
        print("Starting the software")
        self.software_running = True
        self.parameters = parameters
        # Create instances of the main classes
        self.fleet_manager = fleet_manager(self.parameters)
        self.map_builder = mapbuilder(self.parameters)
        self.graphical_interface = graphical_interface(self.parameters, 
                                                       fleet_manager = self.fleet_manager, 
                                                       map_builder = self.map_builder,
                                                       get_static_snapshot = self.static_snapshot,
                                                       get_dynamic_snapshot = self.dynamic_snapshot)
        
        # Start everything in separate threads
        self.intialize_thread()
        self.graphical_interface.run()
        
    def physics_loop(self):
        while self.software_running:
            for USV in self.fleet_manager.USV.values():
                USV.physic_step(self.parameters["dt"])
            time.sleep(self.parameters["dt"])

    def initialize_USVs(self):
        # input("Appuie sur Entrée pour tester les moteurs...")
        self.thread_fleet_dict = {}
        for i in range(1, self.fleet_manager.nb_USVs + 1):
            self.thread_fleet_dict[i] = threading.Thread(target=self.fleet_manager.USV[i].do.wrapper, daemon=True)
            self.thread_fleet_dict[i].start()
    
    def intialize_thread(self):
        
        self.thread_IHM = threading.Thread(target=self.initialize_USVs, daemon=True)
        self.thread_IHM.start()

        self.thread_physics_loop = threading.Thread(target=self.physics_loop, daemon=True)
        self.thread_physics_loop.start()

    def static_snapshot(self):
        """
        Creates a dict with allthe map informaation for the simulation. sent at the beginning onl once. Useful for visualization.
        """
        list_waypoints = []
        for waypoint_id, waypoint in self.map_builder.waypoint.dico_of_waypoints.items():
            list_waypoints.append({"id": waypoint_id,
                                   "pose": {"x": waypoint[0],
                                            "y": waypoint[1]}})

        list_obstacles = []
        for obstacle_id, obstacle in self.map_builder.obstacle.dico_of_obstacles.items():
            list_obstacles.append({"id": obstacle_id,
                                   "pose": {"x": obstacle[0],
                                            "y": obstacle[1],
                                            },
                                    "radius": obstacle[2]})

        return {"waypoints": list_waypoints, "obstacles": list_obstacles}

    def dynamic_snapshot(self, t_s):
        """
        Creates a dict with the state of every asset at the instant it is called. Useful to record simulation logs and for visualization.
        """
        list_assets_states = []
        for asset in self.fleet_manager.list_of_assets:
            list_assets_states.append({"id": asset.id,
                                       "type": asset.type,
                                       "pose": {"x": asset.x,
                                                "y": asset.y,
                                                "psi_rad": asset.psi_rad},
                                     })

        return {"time": t_s,
                "dt" : self.parameters["dt"], 
                "assets_state": list_assets_states}
    
    #TODO: should stop every thread with a stop event.

        
