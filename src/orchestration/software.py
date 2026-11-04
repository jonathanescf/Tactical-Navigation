import threading
import time

from src.simulation.map.map import mapbuilder
from src.graphics.graphical_interface import graphical_interface
from src.simulation.simulator import simulator

class software:

    def __init__(self, parameters:dict):
        self.software_running = True
        self.parameters = parameters
        # Create instances of the main classes
        self.simulator = simulator(self.parameters)
        self.map_builder = mapbuilder(self.parameters)
        self.graphical_interface = graphical_interface(self.parameters, 
                                                       simulator = self.simulator, 
                                                       get_static_snapshot = self.static_snapshot,
                                                       get_dynamic_snapshot = self.dynamic_snapshot)
        
        # Start everything in separate threads
        self.initialize_threads()
        self.graphical_interface.run()
        
    def physics_loop(self):
        while self.software_running:
            for USV in self.simulator.USV.values():
                USV.physic_step(self.parameters["dt"])
            time.sleep(self.parameters["dt"])

    def brain_loop(self):
        while self.software_running:
            for asset in self.simulator.list_of_assets:
                asset.do.brain_step()
            time.sleep(self.parameters["dt"])


    def initialize_USVs(self):
        
        self.thread_brain = threading.Thread(target=self.brain_loop, daemon=True)
        self.thread_brain.start()
        # self.thread_fleet_dict = {}
        # for i in range(1, self.simulator.nb_USVs + 1):
        #     self.thread_fleet_dict[i] = threading.Thread(target=self.simulator.USV[i].do.wrapper, daemon=True)
        #     self.thread_fleet_dict[i].start()
    
    def initialize_threads(self):
        
        self.thread_init_usvs = threading.Thread(target=self.initialize_USVs, daemon=True)
        self.thread_init_usvs.start()

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
        for asset in self.simulator.list_of_assets:
            list_assets_states.append({"id": asset.id,
                                       "type": asset.type,
                                       "pose": {"x": asset.x,
                                                "y": asset.y,
                                                "psi_rad": asset.psi_rad},
                                        "ongoing_target": {"x": asset.do.ongoing_target[0], 
                                                           "y": asset.do.ongoing_target[1]}})

        return {"time": t_s,
                "dt" : self.parameters["dt"], 
                "assets_state": list_assets_states}
    
    #TODO: should stop every thread with a stop event.

        
