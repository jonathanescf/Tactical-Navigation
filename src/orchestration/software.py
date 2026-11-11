import threading
import time
import numpy as np


from src.orchestration.builder import *
from src.graphics.graphical_interface import graphical_interface
from src.autonomy.world_grid import world_grid
from src.autonomy.fleet_supervisor import supervisor
from src.simulation.simulator import simulator

class software:
    def __init__(self, parameters:dict, path_scenario):
        self.initialize(parameters, path_scenario)
    

    def initialize(self, parameters:dict, path_scenario):
        self.software_running = True
        self.parameters = parameters

        scenario = load_scenario(path_scenario)
        self.mission = build_mission(scenario)
        
        self.terrain = build_terrain(scenario)
        self.world_grid = world_grid(self.terrain, precision = 5)
        

        # Create instances of the main classes
        self.fleet = build_fleet(self.parameters, self.terrain)
        self.simulator = simulator(self.terrain, self.fleet.bodies, self.parameters["dt"])
        self.supervisor = supervisor(self.world_grid, self.fleet.brains, self.mission)

        
        # Start everything in separate threads
        self.backend_loop = threading.Thread(target=self.loop, daemon=True)
        self.backend_loop.start()
        
        self.graphical_interface = graphical_interface(self.parameters, 
                                                               get_static_snapshot = self.static_snapshot,
                                                               get_dynamic_snapshot = self.dynamic_snapshot)
        self.graphical_interface.run()

    def loop(self):
        while self.software_running:
            ti_tick = time.perf_counter()  # Start the timer
            self.supervisor.fulfill_mission()
            self.simulator.global_physic_step()
            tf_tick = time.perf_counter()  # End the timer
            time.sleep(self.parameters["dt"] - (tf_tick - ti_tick))

    def static_snapshot(self, grid_n: int = 250) -> dict:
        # --- Terrain : SDF échantillonnée une seule fois ---
        x0, x1, y0, y1 = self.terrain.bounds
        margin = max(self.terrain.base_size) + 10
        xs = np.linspace(x0 - margin, x1 + margin, grid_n)
        ys = np.linspace(y0 - margin, y1 + margin, grid_n)
        XX, YY = np.meshgrid(xs, ys)
        D = self.terrain.distance(XX, YY)

        # --- Costmap : la grille discrète vue par le planificateur ---
        gm = self.world_grid.gridmap                 # (i, j, 2), ligne 0 = haut (Y inversé)
        res = self.world_grid.global_precision
        x_left, y_top = gm[0, 0]
        x_right, y_bottom = gm[-1, -1]

        return {
            "name": self.mission.get("name"),
            "bounds": list(self.terrain.bounds),
            "base_size": list(self.terrain.base_size),
            "arrival_radius": float(self.mission["arrival_radius"]),
            "waypoints": [{"id": i, "x": float(x), "y": float(y)}
                        for i, (x, y) in enumerate(self.mission["waypoints"], start=1)],
            "terrain_grid": {"x": xs.tolist(), "y": ys.tolist(), "d": D.tolist()},
            "costmap": {
                "res": float(res),
                "extent": [float(x_left - res / 2), float(x_right + res / 2),
                        float(y_bottom - res / 2), float(y_top + res / 2)],
                "cost": self.world_grid.costmap.tolist(),    # lignes de haut en bas
            },
        }


    def dynamic_snapshot(self) -> dict:
        assets = []
        for asset_id, body in self.fleet.bodies.items():
            brain = self.fleet.brains[asset_id]

            x, y, psi, Ul, Ur = body.get_space_state()
            bx, by, bpsi, bv = brain.get_nav_state()
            target = brain.ongoing_target_state

            assets.append({
                "id": asset_id,
                "type": self.fleet.dict_of_assets[asset_id].type,
                "truth": {
                    "pose": {"x": float(x), "y": float(y), "psi_rad": float(psi)},
                    "motor_commands": {"Ul": float(Ul), "Ur": float(Ur)},
                },
                "belief": {
                    "estimated_pose": {"x": float(bx), "y": float(by), "psi_rad": float(bpsi)},
                    "estimated_speed": float(bv),
                    "ongoing_target": None if target is None
                                    else {"x": float(target[0]), "y": float(target[1])},
                    "path": [{"x": float(p[0]), "y": float(p[1])} for p in brain.waypoints],
                    "path_index": int(brain.wp_index),
                },
            })

        return {"time": float(self.simulator.t),
                "dt": float(self.parameters["dt"]),
                "assets": assets}
