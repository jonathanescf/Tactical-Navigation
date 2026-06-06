from src.software_launcher import SOFTWARE_launcher


parameters = {
    "dt": 0.05,
    "nb_of_boats": 1,
    "id_boat": 1,
    "nb_of_waypoints": 3,
    "nb_of_obstacles": 0,
    "list_of_waypoints": None,
    "list_of_obstacles": None,
    }

def main():
    software = SOFTWARE_launcher(parameters)
    software.mission_launcher()


if __name__ == "__main__":
    main()
