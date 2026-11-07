import numpy as np
import matplotlib.pyplot as plt
from src.orchestration.config import parameters
from src.autonomy.guidance.guidance import USV_Guidance


def xy(wp):
    """Un waypoint est (x, y, psi, v). Le champ de potentiel n'utilise que (x, y)."""
    return wp[0], wp[1]


def simulate_route(guidance, waypoints, dict_obstacles, start=(0, 0), alpha=0.5,
                   arrival_radius=3.0, max_steps_per_leg=2000):
    """
    Enchaîne les waypoints dans l'ordre du dictionnaire.
    Sortie : une liste de tronçons {"id", "target", "path" (N x 2), "reached"}.
    """
    pos = np.array(start, dtype=float)      # la position PERSISTE d'un tronçon à l'autre
    legs = []

    for wp_id, wp in waypoints.items():
        x_t, y_t = xy(wp)
        target = np.array([x_t, y_t], dtype=float)
        points = [pos.copy()]
        reached = False

        for _ in range(max_steps_per_leg):
            heading, _, _ = guidance.vector_field(pos[0], pos[1], x_t, y_t, dict_obstacles)
            pos = pos + alpha * np.array([np.cos(heading), np.sin(heading)])
            points.append(pos.copy())
            if np.linalg.norm(pos - target) < arrival_radius:
                reached = True
                break

        legs.append({"id": wp_id, "target": target, "path": np.array(points), "reached": reached})
        if not reached:
            break   # comme la vraie mission : un robot coincé ne passe pas au suivant

    return legs


def plot_vector_field(dict_waypoints=None, dict_obstacles=None, field_wp=None,
                      resolution=20, x_lim=(-400, 400), y_lim=(-400, 400),
                      start=(0, 0), alpha=0.5, arrival_radius=3.0, max_steps_per_leg=2000):

    # Défauts résolus à l'appel, pas à l'import.
    dict_waypoints = dict_waypoints or parameters["dict_of_waypoints"]
    dict_obstacles = dict_obstacles or parameters["dict_of_obstacles"]
    if dict_waypoints is None or dict_obstacles is None:
        raise ValueError("Scénario 'random' : les dicts valent None. "
                         "Passe-les explicitement ou choisis un scénario fixe.")

    guidance = USV_Guidance()       # vector_field n'utilise aucun état de self
    if field_wp is None:
        field_wp = next(iter(dict_waypoints))        # par défaut : le premier waypoint
    x_f, y_f = xy(dict_waypoints[field_wp])

    # --- Champ affiché : il dépend du waypoint visé (field_wp) ---
    X = np.arange(x_lim[0], x_lim[1], resolution)
    Y = np.arange(y_lim[0], y_lim[1], resolution)
    XX, YY = np.meshgrid(X, Y)
    U_grid = np.zeros(XX.shape)
    FX = np.zeros(XX.shape)
    FY = np.zeros(XX.shape)
    k_obstacle_influence = 3

    for i in range(len(Y)):
        for j in range(len(X)):
            heading, U_dict, k_obstacle_influence = guidance.vector_field(
                X[j], Y[i], x_f, y_f, dict_obstacles)
            U_grid[i, j] = sum(U_dict.values())
            FX[i, j] = 10 * np.cos(heading)
            FY[i, j] = 10 * np.sin(heading)

    # --- Trajectoire : tous les waypoints à la suite ---
    legs = simulate_route(guidance, dict_waypoints, dict_obstacles, start, alpha,
                          arrival_radius, max_steps_per_leg)
    for leg in legs:
        print(f"WP {leg['id']}: {len(leg['path']) - 1} pas, atteint = {leg['reached']}")

    # --- Figure ---
    fig = plt.figure(figsize=(12, 6))
    ax1 = fig.add_subplot(121)
    ax2 = fig.add_subplot(122, projection='3d')

    ax1.quiver(XX, YY, FX, FY, angles='xy', scale_units='xy', scale=1,
               color='steelblue', alpha=0.4)

    colors = plt.cm.viridis(np.linspace(0, 1, len(dict_waypoints)))
    for k, leg in enumerate(legs):
        ax1.plot(leg["path"][:, 0], leg["path"][:, 1], '-', color=colors[k], linewidth=2)
    ax1.plot(*start, 'ro', markersize=8, label='Start')

    done = {leg["id"]: leg["reached"] for leg in legs}
    for wp_id, wp in dict_waypoints.items():
        x, y = xy(wp)
        if wp_id not in done:
            style, color = 'o', 'gray'       # pas tenté (mission arrêtée avant)
        elif done[wp_id]:
            style, color = '*', 'green'
        else:
            style, color = 'X', 'red'        # tenté mais pas atteint
        ax1.plot(x, y, style, color=color, markersize=14)
        ax1.annotate(str(wp_id), (x, y), textcoords="offset points", xytext=(6, 6))

    for _, (x_obs, y_obs, r_obs) in dict_obstacles.items():
        ax1.add_patch(plt.Circle((x_obs, y_obs), r_obs, color='red', alpha=0.5))
        ax1.add_patch(plt.Circle((x_obs, y_obs), k_obstacle_influence * r_obs,
                                 color='orange', alpha=0.15))

    ax1.set_xlim(*x_lim)
    ax1.set_ylim(*y_lim)
    ax1.set_aspect('equal')
    ax1.grid(True)
    ax1.set_title(f"Route (champ affiché : vers WP {field_wp})")
    ax1.legend(fontsize=7)

    U_clip = np.clip(U_grid, 0, np.percentile(U_grid, 95))
    ax2.plot_surface(XX, YY, U_clip, cmap='plasma', alpha=0.85)
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('U')
    ax2.set_title(f"Potential surface (WP {field_wp})")

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    plot_vector_field()