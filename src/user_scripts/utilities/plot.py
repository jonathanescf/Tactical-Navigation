from src.user_scripts.utilities.utils import *
from src.user_scripts.src.USV_algorithms import vector_field
from src.user_scripts.config import parameters

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np



def plot_vector_field(x_target=20, y_target=20, dict_obstacles=parameters["dict_of_obstacles"], resolution=20):

    X = list(range(-400, 400, resolution))
    Y = list(range(-400, 400, resolution))
    d = {}
    U_grid = np.zeros((len(Y), len(X)))

    for i, y in enumerate(Y):
        for j, x in enumerate(X):
            F, U, k_obstacle_influence = vector_field(x, y, x_target, y_target, dict_obstacles)
            U_grid[i, j] = sum(U.values())
            norm = np.linalg.norm(F)
            d[(x, y)] = 10 * F / norm if norm > 0 else np.zeros(2)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))

    # --- Graphe 1 : champ de vecteurs ---
    for (x, y), F in d.items():
        ax1.quiver(x, y, F[0], F[1], angles='xy', scale_units='xy', scale=1, color='blue', alpha=0.6)

    ax1.plot(x_target, y_target, 'g*', markersize=15, label='Target')
    for _, (x_obs, y_obs, r_obs) in dict_obstacles.items():
        ax1.add_patch(plt.Circle((x_obs, y_obs), r_obs, color='red', alpha=0.4))
        ax1.add_patch(plt.Circle((x_obs, y_obs), k_obstacle_influence*r_obs, color='orange', alpha=0.15))

    ax1.set_xlim(-400, 400)
    ax1.set_ylim(-400, 400)
    ax1.set_title("Champ de potentiel")
    ax1.grid(True)
    ax1.set_aspect('equal')
    ax1.legend()

    # --- Graphe 2 : surface U(x, y) ---
    XX, YY = np.meshgrid(X, Y)
    ax2 = fig.add_subplot(122, projection='3d')
    ax2.plot_surface(XX, YY, U_grid, cmap='plasma', alpha=0.8)
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('U')
    ax2.set_title("Surface du potentiel répulsif")

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    plot_vector_field()