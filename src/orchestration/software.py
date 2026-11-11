import threading
import time


from src.orchestration.builder import *
from src.graphics.graphical_interface import graphical_interface
from src.autonomy.world_grid import world_grid
from src.autonomy.fleet_supervisor import supervisor

def software(parameters:dict, path_scenario):
    scenario = load_scenario(path_scenario)
    mission = build_mission(scenario)

    terrain = build_terrain(scenario)
    world_grid = world_grid(terrain, precision = 15)

    fleet = build_fleet(parameters, terrain)
    simulator = simulator(terrain, fleet.bodies, parameters["dt"])
    supervisor = supervisor(world_grid, mission, fleet.brains)

    def __init__(self, parameters:dict, path_scenario):
        self.software_running = True
        self.parameters = parameters

        scenario = load_scenario(path_scenario)
        self.mission_info = build_mission(scenario)
        self.terrain = build_terrain(scenario)
        self.world_grid = world_grid(self.terrain, precision = 15)
        
 

        # Create instances of the main classes
        self.fleet = build_fleet(self.parameters, self.terrain)
        self.supervisor = supervisor(self.world_grid, self.mission_info, self.fleet.brains)

        
        
        # Start everything in separate threads
        self.thread_mission_loop = threading.Thread(target=self.loop, daemon=True)
        self.thread_mission_loop.start()
        
        self.graphical_interface = graphical_interface(self.parameters, 
                                                               get_static_snapshot = self.static_snapshot,
                                                               get_dynamic_snapshot = self.dynamic_snapshot)
        self.graphical_interface.run()

    def loop(self):
        while self.software_running:
            for asset_body in self.fleet.bodies.values():
                asset_body.physic_step(self.parameters["dt"])
            self.supervisor.fulfill_mission()
            time.sleep(self.parameters["dt"])

    def static_snapshot(self):
        return {
            "bounds": self.terrain.bounds,
            "base_size": self.terrain.base_size,
            "waypoints": [{"id": i, "pose": {"x": x, "y": y}}
                        for i, (x, y) in enumerate(self.mission_info["waypoints"], start=1)],
            "terrain_distance": self.terrain.distance,  # a modifier lorsque le gui sera sur web 
                                                        # (impossible de serialiser une fonction comme ca)
        }

    def dynamic_snapshot(self, t_s):
        """
        Creates a dict with the state of every asset at the instant it is called. Useful to record simulation logs and for visualization.
        """
        list_assets_states = []
        for asset in self.fleet.dict_of_assets.values():
            x, y, psi_rad, Ul, Ur = asset.usv_body.get_space_state()
            if asset.usv_brain.ongoing_target_state is None:
                ongoing = None
            else:
                ongoing = {"x": asset.usv_brain.ongoing_target_state[0], 
                           "y": asset.usv_brain.ongoing_target_state[1]}
            list_assets_states.append({"id": asset.id,
                                       "type": asset.type,
                                       "pose": {"x": x,
                                                "y": y,
                                                "psi_rad": psi_rad},
                                        "motor_commands": {"Ul": Ul,
                                                           "Ur": Ur},
                                        "ongoing_target": ongoing})

        return {"time": t_s,
                "dt" : self.parameters["dt"], 
                "assets_state": list_assets_states}
