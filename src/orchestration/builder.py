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


def plot_scenario():
    pass


if __name__== "__main__":

    path = "scenarios/obstacle_avoidance.toml"

    scenario_info = load_scenario(path)

    terrain = build_terrain(scenario_info)
    mission_info = build_mission(scenario_info)

    terrain.plot_terrain() #plot scenario a l avenir
