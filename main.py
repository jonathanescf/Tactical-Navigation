from src.orchestration.software import software
from src.orchestration.config import parameters

path_a_star_scenario = "scenarios/a_star.toml"
path_direct_waypoints_scenario = "scenarios/direct_waypoints.toml"
path_obstacle_avoidance_scenario = "scenarios/obstacle_avoidance.toml"


def main():
    path_scenario = path_a_star_scenario
    tactical_nav = software(parameters, path_scenario)


if __name__ == "__main__":
    main()
