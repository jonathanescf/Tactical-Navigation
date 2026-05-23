import threading

from src.simulation import simulation
from src.launcher import IHM_launcher
from src.mapbuilder import mapbuilder



def thread_manager(simu, wrapper):
    t_wrapper = threading.Thread(target=wrapper.run, daemon=True)
    t_wrapper.start()

    simu.run()   # bloque ici → thread principal

def main():
    map_build = mapbuilder(nb_of_waypoints=3, nb_of_obstacles=0)
    simu    = simulation(nb_of_boats=1, map_builder=map_build, dt=0.05)
    ihm = IHM_launcher(map_builder = map_build,missions = simu.missions)

    thread_manager(simu, ihm)
    

if __name__ == "__main__":
    main()