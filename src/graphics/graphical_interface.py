from collections import deque
from typing import Callable

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

ARROW_LEN = 2.0
TRAIL_LEN = 2000

BG_FIG      = "#0b1320"
BG_AXES     = "#16263a"
OBSTACLE    = "#ff5252"
OBSTACLE_ED = "#ffffff"
WAYPOINT    = "#e0e0e0"
TARGET      = "#ffea00"
TEXT        = "#e6edf3"
ASSET_COLORS = ["#00e5ff", "#ff4081", "#b388ff", "#69f0ae", "#ff9100"]

# coût 0 = eau libre (même couleur que le fond), 255 = bloqué. Reste lisible quand la costmap sera graduée.
COST_CMAP = LinearSegmentedColormap.from_list("cost", [BG_AXES, "#ffd54f", OBSTACLE])


def _tail(pose: dict, length: float = ARROW_LEN) -> tuple[float, float]:
    return (pose["x"] - np.cos(pose["psi_rad"]) * length,
            pose["y"] - np.sin(pose["psi_rad"]) * length)


class graphical_interface:
    def __init__(self, parameters: dict,
                 get_static_snapshot: Callable[[], dict],
                 get_dynamic_snapshot: Callable[[], dict]):
        self.dt = parameters["dt"]
        self.get_static_snapshot = get_static_snapshot
        self.get_dynamic_snapshot = get_dynamic_snapshot
        self.artists: dict[str, dict] = {}
        self.setup()

    # ------------------------------------------------------------ setup
    def setup(self):
        self.setup_display()
        self.setup_map()
        self.setup_assets()

    def setup_display(self):
        self.fig, (self.ax_terrain, self.ax_cost) = plt.subplots(
            1, 2, figsize=(17, 7), sharex=True, sharey=True)   # mêmes limites, zoom synchronisé
        self.fig.patch.set_facecolor(BG_FIG)
        self.axes = (self.ax_terrain, self.ax_cost)
        for ax in self.axes:
            ax.set_facecolor(BG_AXES)
            ax.set_aspect("equal", adjustable="box")
            ax.tick_params(colors=TEXT)
            for s in ax.spines.values():
                s.set_color("#4a5a70")
            ax.grid(True, linestyle="--", alpha=0.25, color="white")
        self.title = self.fig.suptitle("", color=TEXT, fontsize=12)

    def setup_map(self):
        snap = self.get_static_snapshot()
        x0, x1, y0, y1 = snap["bounds"]
        margin = max(snap["base_size"]) + 10

        # --- Vue 1 : terrain continu (SDF échantillonnée) ---
        g = snap["terrain_grid"]
        xs, ys, d = np.asarray(g["x"]), np.asarray(g["y"]), np.asarray(g["d"])
        self.ax_terrain.contourf(xs, ys, d, levels=[-1e9, 0], colors=[OBSTACLE], alpha=0.85, zorder=3)
        self.ax_terrain.contour(xs, ys, d, levels=[0], colors=OBSTACLE_ED, linewidths=1.2, zorder=3)
        self.ax_terrain.set_title("Terrain (SDF)", color=TEXT, fontsize=10)

        # --- Vue 2 : costmap discrète, telle que A* la voit ---
        c = snap["costmap"]
        self.ax_cost.imshow(np.asarray(c["cost"]), extent=c["extent"], origin="upper",
                            cmap=COST_CMAP, vmin=0, vmax=255,
                            interpolation="nearest", zorder=1)   # nearest : on veut voir les cases
        self.ax_cost.set_title(f'Costmap (résolution {c["res"]:g} m)', color=TEXT, fontsize=10)

        # --- Waypoints sur les deux vues ---
        for ax in self.axes:
            for wp in snap["waypoints"]:
                ax.plot(wp["x"], wp["y"], "o", color=WAYPOINT, markersize=7,
                        markeredgecolor="black", zorder=4)
                ax.annotate(str(wp["id"]), (wp["x"], wp["y"]), textcoords="offset points",
                            xytext=(6, 6), color=TEXT, fontsize=9, fontweight="bold", zorder=4)

        self.ax_terrain.set_xlim(x0 - margin, x1 + margin)     # axes partagés : règle les deux
        self.ax_terrain.set_ylim(y0 - margin, y1 + margin)
        self.scenario_name = snap.get("name") or ""

    def _make_artists(self, ax, color: str, pose: dict) -> dict:
        """Un jeu d'artistes pour un asset sur UN axe."""
        arrow = ax.annotate("", xy=(pose["x"], pose["y"]), xytext=_tail(pose), zorder=7,
                            arrowprops=dict(arrowstyle="-|>", color=color, lw=3, mutation_scale=22))
        trail, = ax.plot([], [], "-", color=color, lw=1.8, alpha=0.7, zorder=5)
        path, = ax.plot([], [], "--", color=color, lw=1.5, alpha=0.9, zorder=5)
        estimate, = ax.plot([], [], "x", color="white", markersize=8, markeredgewidth=2, zorder=8)
        target, = ax.plot([], [], "*", color=TARGET, markersize=16, markeredgecolor="black", zorder=8)
        label = ax.annotate("", (0, 0), color=TARGET, fontsize=9, fontweight="bold", zorder=8)
        return {"arrow": arrow, "trail": trail, "path": path,
                "estimate": estimate, "target": target, "target_label": label}

    def setup_assets(self):
        assets = self.get_dynamic_snapshot()["assets"]
        for k, asset in enumerate(assets):
            color = ASSET_COLORS[k % len(ASSET_COLORS)]
            pose = asset["truth"]["pose"]
            self.artists[asset["id"]] = {
                "color": color,
                "history": deque(maxlen=TRAIL_LEN),            # partagé entre les deux vues
                "views": [self._make_artists(ax, color, pose) for ax in self.axes],
            }
        self.setup_legends(assets)

    def setup_legends(self, assets):
        style = dict(facecolor="#1a2a3a", edgecolor="#4a5a70", labelcolor=TEXT, fontsize=8)
        grey = "#9aa7b5"
        ax = self.ax_cost          # légendes à droite de la 2e vue, hors carte

        symbols = [
            Patch(facecolor=OBSTACLE, edgecolor=OBSTACLE_ED, label="Obstacle / case bloquée"),
            Line2D([], [], marker="o", ls="", color=WAYPOINT, markeredgecolor="black", label="Waypoint"),
            Line2D([], [], marker="*", ls="", color=TARGET, markeredgecolor="black",
                   markersize=12, label="Cible courante"),
            Line2D([], [], marker=">", ls="", color=grey, markersize=9, label="Position vraie (cap)"),
            Line2D([], [], marker="x", ls="", color="white", markeredgewidth=2, label="Position estimée"),
            Line2D([], [], ls="-", color=grey, label="Trajectoire parcourue"),
            Line2D([], [], ls="--", color=grey, label="Chemin planifié"),
        ]
        leg1 = ax.legend(handles=symbols, title="Éléments", loc="upper left",
                         bbox_to_anchor=(1.02, 1.0), **style)
        leg1.get_title().set_color(TEXT)
        ax.add_artist(leg1)

        robots = [Line2D([], [], lw=4, color=self.artists[a["id"]]["color"],
                         label=f'{a["type"]} {a["id"]}') for a in assets]
        leg2 = ax.legend(handles=robots, title="Robots", loc="lower left",
                         bbox_to_anchor=(1.02, 0.0), **style)
        leg2.get_title().set_color(TEXT)

    # ----------------------------------------------------------- update
    def run(self):
        self.ani = animation.FuncAnimation(
            self.fig, self.update, interval=int(self.dt * 1000),
            blit=False, cache_frame_data=False)
        plt.tight_layout()
        plt.show()

    def update(self, _):
        snap = self.get_dynamic_snapshot()
        self.title.set_text(f"{self.scenario_name}   t = {snap['time']:.1f} s")
        for asset in snap["assets"]:
            self.update_asset(self.artists[asset["id"]], asset)

    def update_asset(self, entry: dict, asset: dict):
        truth, belief = asset["truth"]["pose"], asset["belief"]
        entry["history"].append((truth["x"], truth["y"]))
        trail_x, trail_y = zip(*entry["history"])
        est = belief["estimated_pose"]
        path = belief["path"]
        px, py = [p["x"] for p in path], [p["y"] for p in path]
        t = belief["ongoing_target"]

        for art in entry["views"]:                       # même contenu dans les deux vues
            art["arrow"].set_position(_tail(truth))
            art["arrow"].xy = (truth["x"], truth["y"])
            art["trail"].set_data(trail_x, trail_y)
            art["estimate"].set_data([est["x"]], [est["y"]])
            art["path"].set_data(px, py)
            if t is None:
                art["target"].set_data([], [])
                art["target_label"].set_visible(False)
            else:
                art["target"].set_data([t["x"]], [t["y"]])
                art["target_label"].set_text(f'TARGET {asset["id"]}')
                art["target_label"].set_position((t["x"], t["y"] - 3))
                art["target_label"].set_visible(True)