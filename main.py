from src.orchestration.software import software
from src.orchestration.config import parameters




def main():
    path_scenario = "scenarios/direct_waypoints.toml"
    tactical_nav = software(parameters, path_scenario)


if __name__ == "__main__":
    main()
