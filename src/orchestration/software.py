import threading
import time
import numpy as np


from src.orchestration.builder import *
from src.graphics.graphical_interface import graphical_interface
from src.autonomy.world_grid import world_grid
from src.autonomy.fleet_supervisor import supervisor
from src.simulation.simulator import simulator
from src.simulation.clock import clock


class software:
    def __init__(self, parameters, path_scenario, headless=False, max_ticks=None):
        self.engine = engine(parameters, path_scenario, publish = not headless)
        self.max_ticks = max_ticks
        self.trace=[]
        self.software_running = True
        if headless:
            self.run_headless()
        else: 
            self.run_gui()

    def run_headless(self):
        for i in range(self.max_ticks):
            self.engine.step()

    def run_gui(self):
        self.backend_loop = threading.Thread(target=self.loop, daemon=True)
        self.graphical_interface = graphical_interface(self.engine.parameters, 
                                                       lambda: self.engine.latest_static_snapshot,
                                                       lambda: self.engine.latest_dynamic_snapshot)
        self.backend_loop.start()
        self.graphical_interface.run()

    def loop(self):
        deadline = time.monotonic()
        while self.software_running:
            self.engine.step()
            deadline += self.engine.clock.dt
            delay = deadline - time.monotonic()
            time.sleep(max(0, delay))


class engine:
    def __init__(self, parameters, path_scenario, publish = False): 
        self.parameters = parameters   
        scenario = load_scenario(path_scenario)
        self.publish = publish
        self.latest_static_snapshot = None
        self.latest_dynamic_snapshot = None 

        # Create instances of the main classes
        self.clock = clock(self.parameters["dt"])
        self.terrain, self.gridmap_precision = build_terrain(scenario), 0.5
        self.gridmap = self.terrain.discrete_terrain(precision=self.gridmap_precision)
        self.mission = build_mission(scenario)
        self.fleet = build_fleet(self.parameters, self.terrain)
        self.supervisor = supervisor(self.gridmap, self.gridmap_precision, self.fleet.brains, self.mission)
        self.simulator = simulator(self.fleet.bodies, self.parameters)

        #initalizes all attribuites at t=0
        self.prime()

    def prime(self):
        self.fleet.update_sensor_step(0)
        if self.publish:
            self.latest_static_snapshot = self.static_snapshot()
            self.latest_dynamic_snapshot = self.dynamic_snapshot() 

    def step(self):
            self.fleet.update_sensor_step(self.clock.tick)
            self.supervisor.fulfill_mission_step()
            self.simulator.global_physic_step()
            self.clock.advance()
            self.record_trace
            if self.publish:
                self.latest_dynamic_snapshot = self.dynamic_snapshot() 
            
     
    def static_snapshot(self, grid_n: int = 250) -> dict:
        # --- Terrain : SDF échantillonnée une seule fois ---
        x0, x1, y0, y1 = self.terrain.bounds
        margin = max(self.terrain.base_size) + 10
        xs = np.linspace(x0 - margin, x1 + margin, grid_n)
        ys = np.linspace(y0 - margin, y1 + margin, grid_n)
        XX, YY = np.meshgrid(xs, ys)
        D = self.terrain.distance(XX, YY)

        # --- Costmap : la grille discrète vue par le planificateur ---
        gm = self.gridmap                # (i, j, 2), ligne 0 = haut (Y inversé)
        res = self.gridmap_precision
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
            "costmap": { #TODO! changer le nom de cle: c est pas la costmap la gridmap pour l instant
                "res": float(res),
                "extent": [float(x_left - res / 2), float(x_right + res / 2),
                        float(y_bottom - res / 2), float(y_top + res / 2)],
                "cost": self.gridmap.tolist(),    # lignes de haut en bas
            },
        }

    def dynamic_snapshot(self) -> dict:
        assets = []
        for asset_id, body in self.fleet.bodies.items():
            brain = self.fleet.brains[asset_id]

            x, y, psi_rad, u, v, r, Ul, Ur, Tl, Tr = body.get_space_state()
            x_hat, y_hat, psi_hat, u_hat, v_hat, r_hat = brain.get_nav_state()
            u_des, psi_des = brain.u_des, brain.psi_des
            target = brain.ongoing_target_state

            if x_hat is not None: 
                estimated_pose = {"x": float(x_hat), "y": float(y_hat), "psi_rad": float(psi_hat)}
            else:
                estimated_pose = {"x": float(0), "y": float(0), "psi_rad": float(0)}

            assets.append({
                "id": asset_id,
                "type": self.fleet.dict_of_assets[asset_id].type,
                "truth": {
                    "pose": {"x": float(x), "y": float(y), "psi_rad": float(psi_rad)},
                    "speed": {"u": float(u), "v": float(v), "r": float(r)},
                    "motor_commands": {"Ul": float(Ul), "Ur": float(Ur)},
                    "Thrust": {"Tl": float(Tl), "Tr": float(Tr)},
                },
                "belief": {
                    "estimated_pose": estimated_pose,
                    "estimated_speed": {"u_hat": float(u_hat), "v_hat": float(v_hat), "r_hat": float(r_hat)},
                    "desired_speed": float(u_des),
                    "desired_heading": float(psi_des),
                    "ongoing_target": None if target is None
                                    else {"x": float(target[0]), "y": float(target[1])},
                    "path": [{"x": float(p[0]), "y": float(p[1])} for p in brain.waypoints],
                    "path_index": int(brain.wp_index),
                },
            })

        return {"k": float(self.clock.tick),
                "time": float(self.clock.t),
                "dt": float(self.clock.dt),
                "assets": assets}

    def record_trace(self):
        for asset_id in sorted(self.fleet.bodies):
            x, y, psi, u, v, r, Ul, Ur, Tl, Tr = self.fleet.bodies[asset_id].get_space_state()
            self.trace.append((self.clock.tick, x, y, psi, u, v, r, Ul, Ur, Tl, Tr))
