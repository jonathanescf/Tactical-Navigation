
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider
from src.user_scripts.config import parameters
from src.user_scripts.utilities.utils import *
from src.fleet.USV import USV

def plot_vector_field(x_target=-100, y_target=250, dict_obstacles=parameters["dict_of_obstacles"],
                      resolution=20, x_lim=(-400, 400), y_lim=(-400, 400),
                      x_start=300, y_start=-350, alpha=0.5, nb_steps=2000):

    X = list(range(x_lim[0], x_lim[1], resolution))
    Y = list(range(y_lim[0], y_lim[1], resolution))

    usv = USV(id=1)
    d_vectors = {}
    U_grid    = np.zeros((len(Y), len(X)))
    k_obstacle_influence = 3

    # Calcul du champ
    for i, y in enumerate(Y):
        for j, x in enumerate(X):
            heading, U_dict, k_obstacle_influence = usv.do.vector_field(x_USV=x, y_USV=y, x_target=x_target, y_target=y_target, dict_obstacles=dict_obstacles)
            U_grid[i, j] = sum(U_dict.values())
            F = np.array([np.cos(heading), np.sin(heading)])
            d_vectors[(x, y)] = 10 * F

    # Trajectoire simulée
    pos        = np.array([float(x_start), float(y_start)])
    trajectory = [pos.copy()]
    for _ in range(nb_steps):
        heading, _, _ = usv.do.vector_field(pos[0], pos[1], x_target, y_target, dict_obstacles)
        F   = np.array([np.cos(heading), np.sin(heading)])
        pos = pos + alpha * F
        trajectory.append(pos.copy())
        if np.linalg.norm(pos - np.array([x_target, y_target])) < 1:
            break
    trajectory = np.array(trajectory)

    # Figure
    fig = plt.figure(figsize=(10, 5))
    ax1 = fig.add_subplot(121)
    ax2 = fig.add_subplot(122, projection='3d')

    # Champ de vecteurs
    for (x, y), F in d_vectors.items():
        ax1.quiver(x, y, F[0], F[1], angles='xy', scale_units='xy', scale=1, color='steelblue', alpha=0.6)

    # Trajectoire
    ax1.plot(trajectory[:, 0], trajectory[:, 1], 'b-', linewidth=2, label='Trajectory')
    ax1.plot(*trajectory[0], 'ro', markersize=8, label='Starting position')

    # Target
    ax1.plot(x_target, y_target, 'g*', markersize=15, label='Target')

    # Obstacles
    for _, (x_obs, y_obs, r_obs) in dict_obstacles.items():
        ax1.add_patch(plt.Circle((x_obs, y_obs), r_obs, color='red', alpha=0.5))
        ax1.add_patch(plt.Circle((x_obs, y_obs), k_obstacle_influence * r_obs, color='orange', alpha=0.15))

    ax1.set_xlim(*x_lim)
    ax1.set_ylim(*y_lim)
    ax1.set_title("Potential Field")
    ax1.set_aspect('equal')
    ax1.grid(True)
    ax1.legend(fontsize=7)

    # Surface 3D
    XX, YY = np.meshgrid(X, Y)
    U_clip = np.clip(U_grid, 0, np.percentile(U_grid, 95))
    ax2.plot_surface(XX, YY, U_clip, cmap='plasma', alpha=0.85)
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('U')
    ax2.set_title("Potential Surface")

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    plot_vector_field()