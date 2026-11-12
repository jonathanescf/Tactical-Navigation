from src.autonomy.brain import USV_brain
from src.orchestration.fleet import fleet, USV_system

from src.simulation.fleet.hardware.fake_IMU_driver import fake_USV_imu
from src.simulation.fleet.hardware.fake_GPS_driver import fake_USV_gps
from src.simulation.fleet.hardware.fake_USV_motor_driver import fake_USV_ardu
from src.simulation.fleet.vehicles.USV import USV_body

from src.simulation.map.terrain import terrain
import tomllib

def load_scenario(path)-> dict:
    with open(path, "rb") as f:
        info = tomllib.load(f)
        return info

def build_terrain(scenario_info:dict):
    terrain_info = scenario_info["terrain"]
    
    dict_obs = {"circle": [], "rectangle": []}
    base_size = tuple(terrain_info["base_size"])
    dict_obs["circle"] = [tuple(c) for c in terrain_info["circles"]]
    dict_obs["rectangle"] = [tuple(tuple(p) for p in r) for r in terrain_info["rectangles"]]

    return terrain(tuple(terrain_info["bounds"]), dict_obs, base_size)

def build_mission(scenario_info:dict):
    mission_info=scenario_info["mission"]
    mission_info["waypoints"] = [tuple(wpt) for wpt in mission_info["waypoints"]]
    return mission_info

def build_fleet(parameters:dict, terrain):
    asset_dict = {}

    nb_of_usvs = parameters["nb_of_USVs"]
    for num in range(1, nb_of_usvs+1):
        id = "usv" + str(num)
        type = "USV"
        body = USV_body(terrain = terrain) # here
        gnss, imu, motor = fake_USV_gps(USV=body), fake_USV_imu(USV=body), fake_USV_ardu(USV=body)
        brain = USV_brain(ardu=motor, gps=gnss, imu=imu, max_speed=body.max_speed)
        usv = USV_system(id=id, type=type, usv_body=body, usv_brain=brain, usv_gnss=gnss, usv_imu=imu, usv_motor=motor)
        asset_dict[id] = usv

    return fleet(asset_dict)


if __name__== "__main__":

    path = "scenarios/obstacle_avoidance.toml"

    scenario_info = load_scenario(path)

    terrain = build_terrain(scenario_info)
    mission_info = build_mission(scenario_info)

    terrain.plot_terrain() #plot scenario a l avenir
