from src.user_scripts.utilities.utils import *
from src.user_scripts.src.USV_algorithms import vector_field
from src.user_scripts.config import parameters

import matplotlib.pyplot as plt
import numpy as np



def plot_vector_field(x_target=20, y_target=20, dict_obstacles=parameters["dict_of_obstacles"], resolution=5):

    X = range(-100, 100, resolution)
    Y = range(-100, 100, resolution)
    d = {}

    for x in X:
        for y in Y:
            F, k_obstacle_influence = vector_field(x, y, x_target, y_target, dict_obstacles)
            norm = np.linalg.norm(F)
            if norm > 0:
                d[(x, y)] = 1 * F / norm  # normalisé à 1
            else:
                d[(x, y)] = np.zeros(2)

    # Affichage des flèches
    plt.figure(figsize=(8, 8))
    for (x, y), F in d.items():
        plt.quiver(x, y, F[0], F[1], angles='xy', scale_units='xy', scale=1, color='blue', alpha=0.6)

    # Target
    plt.plot(x_target, y_target, 'g*', markersize=15, label='Target')

    # Obstacles
    for _, (x_obs, y_obs, r_obs) in dict_obstacles.items():
        plt.gca().add_patch(plt.Circle((x_obs, y_obs), r_obs, color='red', alpha=0.4))
        plt.gca().add_patch(plt.Circle((x_obs, y_obs), k_obstacle_influence*r_obs, color='orange', alpha=0.15))

    plt.xlim(-100, 100)
    plt.ylim(-100, 100)
    plt.title("Champ de potentiel")
    plt.grid(True)
    plt.axis('equal')
    plt.show()


if __name__ == "__main__":
    plot_vector_field()