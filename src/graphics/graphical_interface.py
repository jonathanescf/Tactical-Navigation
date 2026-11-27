import multiprocessing as mp
import queue
import time
from collections import deque
from types import SimpleNamespace
from typing import Callable
 
import matplotlib
matplotlib.use("QtAgg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.artist import Artist
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.lines import Line2D
from matplotlib.patches import Patch, Polygon, Rectangle
from matplotlib.ticker import MaxNLocator
from matplotlib.widgets import Button

# =====================================================================
#  CHOIX DE LA VERSION
# =====================================================================
MINIMAL_GUI = False      # True : GUI minimale (carte + costmap + robot) optimisée pour la fluidité
                         # False : GUI élaborée (panneau latéral, graphes, légende, boutons...)

# Version élaborée uniquement (ignoré si MINIMAL_GUI) :
SEPARATE_PANEL = True    # True  : le panneau est dans une 2e fenêtre, pilotée par un 2e PROCESSUS
                         #         (la carte n'est plus jamais ralentie par le rendu du panneau)
                         # False : tout dans une seule fenêtre (blitting de la carte + panneau à PANEL_FPS)
AUTO_PLACE_WINDOWS = True   # en mode séparé : place les 2 fenêtres côte à côte (Qt)
WINDOW_TITLEBAR_PX = 40     # marge retranchée à la hauteur de l'écran (barre de titre des fenêtres)
IPC_QUEUE_MAX = 256         # taille max de la file carte -> panneau (au-delà, les snapshots sont abandonnés)

# ---- Cadences de rendu ----
RENDER_FPS_DEFAULT = 30       # cadence de la carte (surchargeable par parameters["render_fps"])
PANEL_FPS = 5                 # panneau, fenêtre unique (le rendu complet fait saccader la carte)
PANEL_FPS_SEPARATE = 10       # panneau, mode séparé (n'affecte plus la carte : tu peux monter plus haut)
                              # surchargeable par parameters["panel_fps"]

# ---- Options de la version minimale ----
MIN_FPS = 30                  # cadence de rendu en mode minimal
MIN_DEFAULT_VIEW = "terrain"  # couche affichée au démarrage : "terrain" ou "costmap" (touche 'm' pour basculer)
MIN_HULL_LEN = 5.0            # longueur du triangle représentant l'USV (m)
MIN_SHOW_TRAIL = False        # trace du trajet parcouru (un peu plus coûteux)
MIN_TRAIL_LEN = 300           # nombre de points de la trace en mode minimal
MIN_SHOW_PATH = False         # trajet planifié + cible courante
 
ARROW_LEN = 5.0
EST_ARROW_LEN = 4.0
DES_ARROW_LEN = 12.0     # longueur de la flèche de cap désiré sur la carte (m)
TRAIL_LEN = 2000
BUTTON_H = 0.032         # hauteur (fraction de la fenêtre) de la rangée boutons / horloge au-dessus de la carte
LEGEND_H = 0.085         # hauteur (fraction de la fenêtre) de la légende sous la carte
 
# ---- Options d'interface ----
FULLSCREEN = False        # plein écran à l'ouverture ('f' pour basculer ensuite) ; fenêtre unique seulement
SIDE_WIDTH = 1.75        # largeur du panneau latéral relative à la carte (qui vaut 2.0)
RADIO_X = 0.42           # début (fraction de la largeur du panneau) de la zone des boutons radio dans l'en-tête
MOTOR_LIM = 1.0          # amplitude initiale des commandes moteurs Ul/Ur (s'élargit toute seule si dépassée)
THRUST_LIM = 1.0         # amplitude initiale des poussées Tl/Tr (s'élargit toute seule si dépassée)
THRUST_UNIT = "N"        # unité affichée pour la poussée (à adapter à ton modèle)
SPEED_MAX = 5.0          # échelle initiale de la jauge / du graphe de vitesse, en m/s (s'élargit toute seule)
SPEED_TOL = 0.25         # écart |v - consigne| (m/s) en dessous duquel l'écart s'affiche en "OK"
HISTORY_S = 30.0         # fenêtre glissante des mini-graphes temporels, en secondes
BAR_HEADROOM = 1.25      # marge au-dessus des barres pour afficher les valeurs
SETPOINT_KEY = "desired_speed"      # clé (dans belief, ou à la racine de l'asset) de la vitesse désirée, optionnelle
HEADING_KEY = "desired_heading"     # idem pour le cap désiré (en radians, repère monde), optionnel
HEADING_TOL_DEG = 5.0               # écart de cap (°) en dessous duquel il s'affiche en "OK"
MS_TO_KN = 1.0 / 0.514444
 
# ---- Vue de suivi ----
FOLLOW_W0 = 80.0         # largeur de vue (m) à l'activation du suivi
ZOOM_STEP = 1.4          # facteur appliqué par clic sur + / −
ZOOM_MIN_W = 30.0        # largeur de vue minimale (m) ; > 2 × DES_ARROW_LEN pour garder la flèche de cap visible
 
# ---- Palette : carte marine de nuit, teintes naturelles et désaturées ----
BG_FIG      = "#15191d"   # fond de fenêtre : graphite
BG_AXES     = "#1f3440"   # eau : gris-bleu ardoise
BG_CARD     = "#1b2025"   # fond des cartes USV
OBSTACLE    = "#5b5240"   # terre : brun-gris kaki
OBSTACLE_ED = "#cfc6b0"   # trait de côte : sable clair
WAYPOINT    = "#d5d9dc"
TARGET      = "#d1a850"   # cible / consigne : ocre doré
TEXT        = "#d9dee2"
TEXT_DIM    = "#8b98a3"
BORDER      = "#46535e"
BTN_OFF     = "#222a31"
BTN_HOVER   = "#2d3741"
BTN_ON      = "#46708a"   # bleu acier
GAUGE_BG    = "#2a333b"
CMD_L       = "#b86a62"   # commande bâbord (Ul) : brique
CMD_R       = "#6a9e7f"   # commande tribord (Ur) : vert sauge
THR_L       = "#d4a39c"   # poussée bâbord (Tl) : brique clair
THR_R       = "#97c0a6"   # poussée tribord (Tr) : sauge clair
OK_COLOR    = "#74a384"
WARN_COLOR  = "#cf9a5c"
ASSET_COLORS = ["#6fb0c8", "#9d8bb8", "#d7dbe0", "#d09a62", "#c48aa3"]   # bleu acier, mauve, blanc cassé, ambre, rose poudré

# coût 0 = eau libre (même couleur que le fond), 255 = bloqué. Reste lisible quand la costmap sera graduée.
COST_CMAP = LinearSegmentedColormap.from_list("cost", [BG_AXES, "#7d765a", OBSTACLE]) 
 
def _tail(pose: dict, length: float = ARROW_LEN) -> tuple[float, float]:
    return (pose["x"] - np.cos(pose["psi_rad"]) * length,
            pose["y"] - np.sin(pose["psi_rad"]) * length)
 
 
def _layer_artists(cs) -> list:
    """Artistes d'un ContourSet, quelle que soit la version de matplotlib
    (>= 3.8 : le ContourSet est lui-même un Artist ; avant : liste .collections)."""
    return [cs] if isinstance(cs, Artist) else list(cs.collections)
 
 
_NICE_STEPS = (1.0, 1.5, 2.0, 3.0, 5.0, 7.5, 10.0)
 
 
def _nice(x: float) -> float:
    """Valeur « ronde » (1, 1.5, 2, 3, 5, 7.5 × 10^n) au-dessus de x + 10 % de marge,
    pour agrandir proprement une échelle quel que soit son ordre de grandeur."""
    x = max(float(x), 1e-9) * 1.1
    mag = 10.0 ** np.floor(np.log10(x))
    for m in _NICE_STEPS:
        if x <= m * mag:
            return float(m * mag)
    return float(10.0 * mag)
 
 
def _wrap_deg(angle_rad: float) -> float:
    """Angle (rad) ramené dans [-180, 180] et converti en degrés."""
    return float(np.degrees((angle_rad + np.pi) % (2.0 * np.pi) - np.pi))
 
 
def _get_setpoint(asset: dict, key: str = SETPOINT_KEY):
    """Consigne du contrôleur (vitesse ou cap) si elle est exposée dans le snapshot, sinon None."""
    for src in (asset.get("belief", {}), asset):
        sp = src.get(key)
        if sp is not None:
            return float(sp)
    return None
 
 
def _get_est_speed(belief: dict) -> tuple[float, float, float]:
    """(û, v̂, r̂) depuis belief["estimated_speed"].
    Accepte le nouveau format dict {"u_hat", "v_hat", "r_hat"} ou l'ancien format (float = û seul)."""
    ev = belief["estimated_speed"]
    if isinstance(ev, dict):
        return float(ev["u_hat"]), float(ev["v_hat"]), float(ev["r_hat"])
    return float(ev), np.nan, np.nan
 
 
def _fmt(val: float, spec: str, suffix: str = "") -> str:
    """Formate une valeur ; « -- » si elle est absente / non finie."""
    return f"{val:{spec}}{suffix}" if np.isfinite(val) else "--"
 
 
def _drift_deg(u: float, v: float) -> float:
    """Angle de dérive β = atan2(v, u) en degrés (0 si quasiment à l'arrêt ou valeur absente)."""
    if not (np.isfinite(u) and np.isfinite(v)) or np.hypot(u, v) <= 0.05:
        return 0.0
    return float(np.degrees(np.arctan2(v, u)))


def _unpack(asset: dict) -> SimpleNamespace:
    """Grandeurs d'un USV extraites du snapshot (utilisé par la carte, les données et le panneau).
    Accepte un asset complet ou allégé (« _path_len » remplace alors le trajet complet)."""
    truth, belief = asset["truth"], asset["belief"]
    spd = truth["speed"]
    u_hat, v_hat, r_hat = _get_est_speed(belief)
    path = belief.get("path")
    n_path = belief["_path_len"] if "_path_len" in belief else (len(path) if path else 0)
    return SimpleNamespace(
        belief=belief, pose=truth["pose"], est=belief["estimated_pose"],
        u=spd["u"], v=spd["v"], r=spd["r"],                       # vérité : avance, dérive, lacet
        ul=truth["motor_commands"]["Ul"], ur=truth["motor_commands"]["Ur"],
        tl=truth["Thrust"]["Tl"], tr=truth["Thrust"]["Tr"],
        u_hat=u_hat, v_hat=v_hat, r_hat=r_hat,                    # estimation : avance, dérive, lacet
        path=path, n_path=n_path, tgt=belief["ongoing_target"],
        sp=_get_setpoint(asset), psi_des=_get_setpoint(asset, HEADING_KEY))


def _light_asset(asset: dict) -> dict:
    """Copie allégée d'un asset pour l'envoi au panneau : le trajet planifié (potentiellement long,
    et inutile au panneau) est remplacé par sa longueur."""
    belief = asset["belief"]
    light_belief = {k: v for k, v in belief.items() if k != "path"}
    light_belief["_path_len"] = len(belief.get("path") or ())
    out = {k: v for k, v in asset.items() if k != "belief"}
    out["belief"] = light_belief
    return out


def _place_window(fig, side: str) -> None:
    """Place la fenêtre dans la moitié gauche ('left') ou droite ('right') de l'écran (backend Qt).
    Les largeurs suivent le gabarit carte / panneau (2.0 / SIDE_WIDTH). Sans effet si l'API est absente."""
    try:
        win = fig.canvas.manager.window
        g = win.screen().availableGeometry()
        frac = 2.0 / (2.0 + SIDE_WIDTH)
        w_left = int(g.width() * frac)
        if side == "left":
            x, w = g.x(), w_left
        else:
            x, w = g.x() + w_left, g.width() - w_left
        win.setGeometry(x, g.y(), w, g.height() - WINDOW_TITLEBAR_PX)
    except Exception:
        pass


class _Blitter:
    """Redessin partiel (blitting). Le reste de la fenêtre (panneau, boutons, légende, et la carte
    statique hors vue de suivi) est mis en cache comme image de fond ; seuls les artistes « animés »
    (USV, flèches, traces, horloge...) sont redessinés à chaque tick.

    Deux niveaux d'animation :
      - artistes animés (add)            : redessinés seuls par-dessus le fond ;
      - axes animés (set_axes_animated)  : redessinés en entier à chaque image. Sert à la carte en vue
        de suivi, dont le contenu bouge (limites changées à chaque image) : le fond en cache ne contient
        alors plus la carte, mais toujours le panneau, la légende et les boutons.

    Le fond est recapturé automatiquement après chaque rendu complet (draw_idle) ; invalidate() force
    ce rendu complet, et un redimensionnement invalide le fond."""

    def __init__(self, canvas):
        self.canvas = canvas
        self.fig = canvas.figure
        self._bg = None
        self._axes: list = []
        self._artists: list = []
        canvas.mpl_connect("draw_event", self._on_draw)
        canvas.mpl_connect("resize_event", self._on_resize)

    def add(self, artist):
        artist.set_animated(True)       # exclu du rendu complet, dessiné uniquement par le blitter
        self._artists.append(artist)
        self._artists.sort(key=lambda a: a.get_zorder())    # tri à l'ajout, pas à chaque image

    def set_axes_animated(self, ax, on: bool):
        """(Dés)active le redessin complet de cet axe à chaque image (le fond en cache est refait)."""
        ax.set_animated(on)
        if on and ax not in self._axes:
            self._axes.append(ax)
        elif not on and ax in self._axes:
            self._axes.remove(ax)
        self.invalidate()

    def invalidate(self):
        """Périme le fond et demande un rendu complet (qui le recapture)."""
        self._bg = None
        self.canvas.draw_idle()

    def _on_resize(self, _event):
        self._bg = None                 # un fond de mauvaise taille ne doit jamais être restauré

    def _on_draw(self, _event):
        self._bg = self.canvas.copy_from_bbox(self.fig.bbox)
        self._draw_animated()

    def _draw_animated(self):
        for ax in self._axes:           # d'abord les axes animés (leurs enfants animés sont exclus)...
            self.fig.draw_artist(ax)
        for a in self._artists:         # ... puis les artistes animés, déjà triés par zorder
            self.fig.draw_artist(a)

    def update(self):
        """Restaure le fond puis redessine uniquement les éléments animés."""
        if self._bg is None:            # pas encore de rendu complet (ou fond périmé) : on le déclenche
            self.canvas.draw_idle()
            return
        self.canvas.restore_region(self._bg)
        self._draw_animated()
        self.canvas.blit(self.fig.bbox)
 
 
class graphical_interface:
    """Supervision des USV. Quatre « rôles » (choisis automatiquement) :
      "min"   : carte minimale seule (MINIMAL_GUI) ;
      "all"   : carte + panneau dans une seule fenêtre (SEPARATE_PANEL = False) ;
      "map"   : fenêtre de la carte, dans le processus principal ; lance le processus du panneau ;
      "panel" : fenêtre du panneau, dans le processus enfant (alimentée par une file de snapshots)."""

    VIEWS = {"terrain": "Terrain (SDF)", "costmap": "Costmap"}
    GAUGE = (0.42, 0.45, 0.54, 0.28)      # x, y, largeur, hauteur de la jauge (coordonnées de l'axe)
 
    def __init__(self, parameters: dict,
                 get_static_snapshot: Callable[[], dict],
                 get_dynamic_snapshot: Callable[[], dict],
                 _role: str = None, _ipc: tuple = None, _colors: dict = None):
        # (_role, _ipc, _colors : usage interne, pour le processus du panneau)
        if _role is not None:
            self.role = _role
        elif MINIMAL_GUI:
            self.role = "min"
        elif SEPARATE_PANEL:
            self.role = "map"
        else:
            self.role = "all"
        self.minimal = self.role == "min"
        self.has_map = self.role != "panel"
        self.has_panel = self.role in ("all", "panel")

        # Période de RENDU de la carte : indépendante du pas de simulation dt (qui appartient au clock de l'engine).
        default_fps = MIN_FPS if self.minimal else RENDER_FPS_DEFAULT
        self.dt = 1.0 / parameters.get("render_fps", default_fps)
        # Période de rafraîchissement du panneau.
        default_panel = PANEL_FPS_SEPARATE if self.role in ("map", "panel") else PANEL_FPS
        self.panel_dt = 1.0 / parameters.get("panel_fps", default_panel)
        self._last_panel_t = 0.0                  # (rôle "all") dernier rafraîchissement du panneau

        self.get_static_snapshot = get_static_snapshot
        self.get_dynamic_snapshot = get_dynamic_snapshot
        self._last_k = None                       # dernier tick de simulation affiché
        self.artists: dict[str, dict] = {}        # un jeu d'artistes carte + données par USV
        self.view_layers: dict[str, list] = {key: [] for key in self.VIEWS}
        self.view_buttons: dict[str, Button] = {}
        self.current_view = "terrain"
        self.selected_id = None                   # USV affiché dans le panneau latéral (boutons radio)
        self.panel: dict = {}                     # artistes du panneau (reconstruits à chaque changement d'USV)
        self._panel_axes: list = []
        self._radio_items: list = []
        self.follow = False                       # vue de suivi active ?
        self.follow_w = FOLLOW_W0                 # largeur de la vue de suivi (m)
        self.zoom_buttons: dict[str, Button] = {}
        self.zoom_axes: dict[str, plt.Axes] = {}
        self.blitter = None                       # créé par setup_display / setup_display_min

        # ---- communication entre les deux processus ----
        self._q_in = self._q_out = None           # file entrante / sortante (vue de CE processus)
        self._proc = None                         # processus du panneau (rôle "map")
        self._panel_dead_reported = False
        self._last_snap = None                    # (rôle "panel") dernier snapshot reçu
        self._init_colors = _colors or {}
        if _ipc is not None:
            self._q_in, self._q_out = _ipc

        self.setup()
        if self.role == "map":
            self._spawn_panel(self.get_dynamic_snapshot())
 
    # ------------------------------------------------------------ setup
    def setup(self):
        snap = self.get_dynamic_snapshot()
        if snap is None or self.get_static_snapshot() is None:
            raise RuntimeError("Aucun snapshot disponible : engine.prime() doit publier "
                               "les snapshots (publish=True) avant la création du GUI.")
        assets = snap["assets"]

        if self.role == "panel":                  # ---- fenêtre du panneau seule ----
            self.setup_display_panel()
            self.setup_panel()
            for asset in assets:
                self.add_asset(asset, self._init_colors.get(asset["id"]))
            return

        if self.minimal:                          # ---- version minimale : carte + robot(s) seulement ----
            self.current_view = MIN_DEFAULT_VIEW
            self.setup_display_min()
            self.setup_map()
            for asset in assets:
                self.add_asset(asset)
            self.set_view(self.current_view)
            return

        self.setup_display()
        self.setup_map()
        self.setup_controls()
        self.setup_legend()
        if self.has_panel:                        # rôle "all" : panneau dans la même fenêtre
            self.setup_panel()
        for asset in assets:
            self.add_asset(asset)
        self.set_view(self.current_view)
        self._hug_map()

    def setup_display_min(self):
        """Gabarit minimal : une seule fenêtre, un seul axe (la carte), une horloge. Aucun bouton, panneau
        ou légende. Les éléments dynamiques sont redessinés par blitting (voir _Blitter)."""
        self.fig, self.ax_map = plt.subplots(figsize=(12, 9))
        self.fig.patch.set_facecolor(BG_FIG)
        try:
            self.fig.canvas.manager.set_window_title("USV Supervision (minimal)")
        except Exception:
            pass
        self.fig.subplots_adjust(left=0.05, right=0.99, bottom=0.05, top=0.94)
        ax = self.ax_map
        ax.set_facecolor(BG_AXES)
        ax.set_aspect("equal", adjustable="box")
        ax.tick_params(colors=TEXT, labelsize=9)
        for s in ax.spines.values():
            s.set_color(BORDER)
        self.title = self.fig.suptitle("", color=TEXT, fontsize=11)
        self.ui_scale = 1.0

        self.blitter = _Blitter(self.fig.canvas)
        self.clock = ax.text(0.01, 0.985, "t = --", transform=ax.transAxes, va="top", ha="left",
                             color=TEXT, fontsize=11, family="monospace", zorder=20,
                             bbox=dict(facecolor=BG_CARD, edgecolor=BORDER, alpha=0.85, pad=3))
        self.blitter.add(self.clock)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
 
    def setup_display(self):
        """Gabarit de la carte : [boutons + horloge | carte | légende]. Rôle "all" : le panneau de l'USV
        sélectionné occupe la colonne de droite de la même fenêtre ; rôle "map" : fenêtre dédiée à la carte
        (le panneau est dans l'autre processus). La carte garde un rapport d'aspect égal et ne remplit donc
        pas toute sa cellule : boutons et légende sont recollés à elle par _hug_map."""
        with_panel = self.has_panel
        self.fig = plt.figure(figsize=(20, 11) if with_panel else (10.5, 11))
        self.fig.patch.set_facecolor(BG_FIG)
        try:
            self.fig.canvas.manager.set_window_title("USV Supervision" if with_panel
                                                     else "USV Supervision · carte")
        except Exception:
            pass
 
        if with_panel:
            outer = self.fig.add_gridspec(1, 2, width_ratios=[2.0, SIDE_WIDTH], wspace=0.06,
                                          left=0.045, right=0.985, bottom=0.04, top=0.945)
            self._side = outer[0, 1]              # panneau latéral : une seule carte, celle de l'USV sélectionné
        else:
            outer = self.fig.add_gridspec(1, 1, left=0.075, right=0.985, bottom=0.04, top=0.945)
        left = outer[0, 0].subgridspec(3, 1, height_ratios=[0.6, 10.0, 1.7], hspace=0.0)
        self.ax_map = self.fig.add_subplot(left[1, 0])
        self.ax_map.set_facecolor(BG_AXES)
        self.ax_map.set_aspect("equal", adjustable="box")
        self.ax_map.tick_params(colors=TEXT, labelsize=10)
        for s in self.ax_map.spines.values():
            s.set_color(BORDER)
        self.ax_map.grid(True, linestyle="--", alpha=0.25, color="white")
 
        self.ui_scale = 1.0
 
        self.title = self.fig.suptitle("", color=TEXT, fontsize=13)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        if with_panel:
            self.fig.canvas.mpl_connect("button_press_event", self._on_click)
        self.fig.canvas.mpl_connect("resize_event", self._hug_map)
        self.blitter = _Blitter(self.fig.canvas)   # redessin partiel de la carte (voir _Blitter)

    def setup_display_panel(self):
        """Fenêtre du panneau (processus enfant) : uniquement l'en-tête + boutons radio, le tableau de bord
        et les graphes de l'USV sélectionné."""
        self.fig = plt.figure(figsize=(9.5, 11))
        self.fig.patch.set_facecolor(BG_FIG)
        try:
            self.fig.canvas.manager.set_window_title("USV Supervision · panneau")
        except Exception:
            pass
        outer = self.fig.add_gridspec(1, 1, left=0.06, right=0.975, bottom=0.04, top=0.965)
        self._side = outer[0, 0]
        self.ui_scale = 1.0
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self.fig.canvas.mpl_connect("button_press_event", self._on_click)
 
    def setup_map(self):
        snap = self.get_static_snapshot()
        x0, x1, y0, y1 = snap["bounds"]
        margin = max(snap["base_size"]) + 10
        ax = self.ax_map
 
        # --- Couche "terrain" : terrain continu (SDF échantillonnée) ---
        g = snap["terrain_grid"]
        xs, ys, d = np.asarray(g["x"]), np.asarray(g["y"]), np.asarray(g["d"])
        fill = ax.contourf(xs, ys, d, levels=[-1e9, 0], colors=[OBSTACLE], alpha=0.85, zorder=3)
        edge = ax.contour(xs, ys, d, levels=[0], colors=OBSTACLE_ED, linewidths=1.2, zorder=3)
        self.view_layers["terrain"] += _layer_artists(fill) + _layer_artists(edge)
 
        # --- Couche "costmap" : grille discrète, telle que A* la voit ---
        c = snap["costmap"]
        im = ax.imshow(np.asarray(c["cost"]), extent=c["extent"], origin="upper",
                       cmap=COST_CMAP, vmin=0, vmax=255,
                       interpolation="nearest", zorder=1)   # nearest : on veut voir les cases
        self.view_layers["costmap"].append(im)
        self.costmap_res = c["res"]
 
        # --- Waypoints (communs aux deux couches) ---
        for wp in snap["waypoints"]:
            ax.plot(wp["x"], wp["y"], "o", color=WAYPOINT, markersize=7,
                    markeredgecolor="black", zorder=4)
            ax.annotate(str(wp["id"]), (wp["x"], wp["y"]), textcoords="offset points",
                        xytext=(6, 6), color=TEXT, fontsize=10, fontweight="bold", zorder=4)
 
        ax.set_xlim(x0 - margin, x1 + margin)
        ax.set_ylim(y0 - margin, y1 + margin)
        # vue d'ensemble mémorisée : sert au retour depuis la vue de suivi et au rapport d'aspect
        self.home_xlim, self.home_ylim = ax.get_xlim(), ax.get_ylim()
        self.home_w = self.home_xlim[1] - self.home_xlim[0]
        self.home_h = self.home_ylim[1] - self.home_ylim[0]
        self.scenario_name = snap.get("name") or ""
        self.title.set_text(self.scenario_name)
 
    def setup_controls(self):
        """Au-dessus de la carte : boutons « segmented control » (couche affichée), bouton « Suivre USV »,
        pile + / − et valeur du zoom, horloge. Leur position exacte est fixée par _hug_map."""
        self.button_axes: dict[str, plt.Axes] = {}
        for key, label in self.VIEWS.items():
            if key == "costmap":
                label += f" ({self.costmap_res:g} m)"
            ax = self.fig.add_axes([0.0, 0.0, 0.1, BUTTON_H])
            btn = Button(ax, label, color=BTN_OFF, hovercolor=BTN_HOVER)
            btn.label.set_color(TEXT)
            btn.label.set_fontsize(11)
            for sp in ax.spines.values():
                sp.set_color(BORDER)
            btn.on_clicked(lambda _evt, key=key: self.set_view(key))
            self.view_buttons[key] = btn          # garder une référence, sinon le bouton est ramassé
            self.button_axes[key] = ax
 
        # ---- Bouton « Suivre USV » (bascule, désélectionnable) ----
        self.ax_follow = self.fig.add_axes([0.0, 0.0, 0.1, BUTTON_H])
        self.follow_button = Button(self.ax_follow, "Suivre USV", color=BTN_OFF, hovercolor=BTN_HOVER)
        self.follow_button.label.set_color(TEXT)
        self.follow_button.label.set_fontsize(11)
        for sp in self.ax_follow.spines.values():
            sp.set_color(BORDER)
        self.follow_button.on_clicked(lambda _evt: self.set_follow(not self.follow))
 
        # ---- Pile verticale + / − ----
        for key, label, factor in (("in", "+", 1.0 / ZOOM_STEP), ("out", "\u2212", ZOOM_STEP)):
            ax = self.fig.add_axes([0.0, 0.0, 0.03, BUTTON_H / 2])
            btn = Button(ax, label, color=BTN_OFF, hovercolor=BTN_HOVER)
            btn.label.set_fontsize(11)
            btn.label.set_fontweight("bold")
            for sp in ax.spines.values():
                sp.set_color(BORDER)
            btn.on_clicked(lambda _evt, f=factor: self.zoom(f))
            self.zoom_buttons[key] = btn
            self.zoom_axes[key] = ax
 
        # ---- Valeur du zoom ----
        self.ax_zoom = self.fig.add_axes([0.0, 0.0, 0.1, BUTTON_H])
        self.ax_zoom.set_facecolor(BG_CARD)
        self.ax_zoom.set_xticks([])
        self.ax_zoom.set_yticks([])
        for sp in self.ax_zoom.spines.values():
            sp.set_color(BORDER)
        self.zoom_text = self.ax_zoom.text(0.5, 0.5, "", transform=self.ax_zoom.transAxes,
                                           ha="center", va="center", color=TEXT, fontsize=8.5,
                                           family="monospace", linespacing=1.0)
 
        # ---- Horloge (animée : mise à jour à la cadence de la carte, pas du panneau) ----
        self.ax_clock = self.fig.add_axes([0.0, 0.0, 0.1, BUTTON_H])
        self.ax_clock.set_facecolor(BG_CARD)
        self.ax_clock.set_xticks([])
        self.ax_clock.set_yticks([])
        for sp in self.ax_clock.spines.values():
            sp.set_color(BORDER)
        self.clock = self.ax_clock.text(0.5, 0.5, "t = --", transform=self.ax_clock.transAxes,
                                        ha="center", va="center", color=TEXT, fontsize=13,
                                        family="monospace", fontweight="bold")
        self.blitter.add(self.clock)
        self._refresh_follow_ui()
 
    def setup_legend(self):
        """Légende sur 3 colonnes, sous la carte (position fixée par _hug_map)."""
        self.ax_leg = self.fig.add_axes([0.0, 0.0, 0.5, LEGEND_H])
        self.ax_leg.axis("off")
        grey = TEXT_DIM
        symbols = [   # remplissage par colonnes : 1-3 carte, 4-6 USV, 7-9 trajets
            Patch(facecolor=OBSTACLE, edgecolor=OBSTACLE_ED, label="Obstacle / blocked cell"),
            Line2D([], [], marker="o", ls="", color=WAYPOINT, markeredgecolor="black", label="Waypoint"),
            Line2D([], [], marker="*", ls="", color=TARGET, markeredgecolor="black",
                   markersize=12, label="Current target"),
            Line2D([], [], marker=">", ls="", color=grey, markersize=9, label="True position (heading)"),
            Line2D([], [], marker="x", ls="", color="white", markeredgewidth=2, label="Estimated position"),
            Line2D([], [], marker=">", ls="", color="white", markersize=6, label="Estimated heading"),
            Line2D([], [], marker=">", ls="", color=TARGET, markersize=6, label="Desired heading"),
            Line2D([], [], ls="-", color=grey, label="Travelled path"),
            Line2D([], [], ls="--", color=grey, label="Planned path"),
        ]
        self.ax_leg.legend(handles=symbols, loc="upper left", ncol=3, mode="expand",
                           borderaxespad=0.0, columnspacing=1.0, handletextpad=0.6, labelspacing=0.6,
                           fontsize=10, facecolor=BTN_OFF, edgecolor=BORDER, labelcolor=TEXT)
 
    def _hug_map(self, _event=None):
        """Recolle la rangée de contrôles au-dessus de la carte et la légende en dessous,
        à la largeur réelle de la carte (appelée au démarrage et à chaque redimensionnement)."""
        self.ax_map.apply_aspect()
        p = self.ax_map.get_position()
        y_top = p.y1 + 0.010
        bw, gap = 0.20 * p.width, 0.012 * p.width
        x = p.x0
        for ax in (*self.button_axes.values(), self.ax_follow):   # Terrain, Costmap, Suivre USV
            ax.set_position([x, y_top, bw, BUTTON_H])
            x += bw + gap
        zw, half = 0.05 * p.width, (BUTTON_H - 0.002) / 2          # pile + / − (deux demi-hauteurs)
        self.zoom_axes["in"].set_position([x, y_top + BUTTON_H - half, zw, half])
        self.zoom_axes["out"].set_position([x, y_top, zw, half])
        x += zw + gap
        vw = 0.11 * p.width                                        # valeur du zoom
        self.ax_zoom.set_position([x, y_top, vw, BUTTON_H])
        x += vw + gap
        self.ax_clock.set_position([x, y_top, max(p.x1 - x, 0.01), BUTTON_H])
        self.ax_leg.set_position([p.x0, p.y0 - 0.050 - LEGEND_H, p.width, LEGEND_H])
 
    # ------------------------------------------------------------ panneau latéral
    def setup_panel(self):
        """Squelette du panneau latéral : en-tête permanent (nom + id de l'USV à gauche, boutons radio
        de sélection à droite) ; le reste (tableau de bord + graphes) est reconstruit par _build_panel."""
        sc = self.ui_scale
        self.card = self._side.subgridspec(3, 1, height_ratios=[0.30, 4.9, 4.5], hspace=0.20)
 
        ax_h = self.fig.add_subplot(self.card[0])
        ax_h.axis("off")
        ax_h.set_xlim(0, 1)
        ax_h.set_ylim(0, 1)
        self.header_text = ax_h.text(0.0, 0.5, "", color=TEXT, fontsize=16 * sc,
                                     fontweight="bold", va="center")
        self.header_line, = ax_h.plot([0, 1], [0, 0], color=TEXT, lw=2.5, solid_capstyle="butt",
                                      clip_on=False)
 
        # boutons radio : axe transparent posé sur la moitié droite de l'en-tête
        pos = self.card[0].get_position(self.fig)
        self.ax_radio = self.fig.add_axes([pos.x0 + RADIO_X * pos.width, pos.y0,
                                           (1.0 - RADIO_X) * pos.width, pos.height])
        self.ax_radio.patch.set_alpha(0.0)
        self._draw_radio()
 
    def _draw_radio(self):
        """Dessine les boutons radio (un par USV, alignés horizontalement, dans la couleur de l'USV)."""
        ax = self.ax_radio
        ax.clear()
        ax.set_axis_off()
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1)
        ax.patch.set_alpha(0.0)
        self._radio_items = list(self.artists)
        n = len(self._radio_items)
        if n == 0:
            return
        cell = 1.0 / n
        fs = 11.0 * self.ui_scale if n <= 4 else 9.0 * self.ui_scale
        for i, aid in enumerate(self._radio_items):
            col = self.artists[aid]["color"]
            active = aid == self.selected_id
            x = i * cell + 0.03 * cell
            ax.plot([x], [0.5], "o", markersize=12, markerfacecolor="none", markeredgecolor=col,
                    markeredgewidth=1.8, clip_on=False)
            if active:
                ax.plot([x], [0.5], "o", markersize=6.5, color=col, clip_on=False)
            ax.text(x + 0.12 * cell, 0.5, str(aid), color=TEXT if active else TEXT_DIM, fontsize=fs,
                    fontweight="bold" if active else "normal", va="center", ha="left", clip_on=False)
 
    def _on_click(self, event):
        """Clic sur un bouton radio de l'en-tête : sélectionne l'USV correspondant."""
        if event.button != 1 or event.x is None or not self._radio_items:
            return
        if not self.ax_radio.bbox.contains(event.x, event.y):
            return
        x = self.ax_radio.transAxes.inverted().transform((event.x, event.y))[0]
        i = int(np.clip(np.floor(x * len(self._radio_items)), 0, len(self._radio_items) - 1))
        self.select_asset(self._radio_items[i])
 
    def select_asset(self, asset_id, notify: bool = True):
        """Sélectionne l'USV affiché dans le panneau et suivi par la caméra. En mode séparé, la sélection
        est transmise à l'autre fenêtre (notify=False pour une sélection reçue de l'autre fenêtre)."""
        if asset_id == self.selected_id or asset_id not in self.artists:
            return
        self.selected_id = asset_id
        if self.has_panel:
            self._build_panel()
            self._draw_radio()
            self._last_panel_t = 0.0                    # (rôle "all") panneau rafraîchi dès le prochain tick
        if self.has_map and self.follow:
            self._apply_follow()
        if self.role == "panel":
            self._panel_refresh()                       # remplit tout de suite le panneau reconstruit
        elif self.blitter is not None:
            self.blitter.invalidate()
        if notify and self._q_out is not None:
            try:
                self._q_out.put_nowait(("select", asset_id))
            except queue.Full:
                pass
 
    def _new_ax(self, spec, **kwargs):
        """Crée un axe du panneau et le mémorise pour pouvoir le détruire au changement d'USV."""
        ax = self.fig.add_subplot(spec, **kwargs)
        self._panel_axes.append(ax)
        return ax
 
    def _build_panel(self):
        """(Re)construit le corps du panneau (tableau de bord + graphes) pour l'USV sélectionné,
        dans sa couleur et avec ses échelles courantes."""
        for ax in self._panel_axes:
            ax.remove()
        self._panel_axes = []
        entry = self.artists[self.selected_id]
        color = entry["color"]
        self.header_text.set_text(f'{entry["type"]}  {self.selected_id}')
        self.header_text.set_color(color)
        self.header_line.set_color(color)
        self.panel = self._make_body(color, entry["lims"])
 
    # ------------------------------------------------------------ USV
    def add_asset(self, asset: dict, color: str = None):
        """Crée les artistes carte (si fenêtre de carte) + les données (historiques, échelles ; si panneau)
        d'un USV, et l'ajoute aux boutons radio. Le premier USV reçu est sélectionné.
        Appelable aussi en cours de simulation."""
        slot = len(self.artists)
        color = color or ASSET_COLORS[slot % len(ASSET_COLORS)]
        pose = asset["truth"]["pose"]
        entry = {"color": color, "type": asset["type"]}

        if self.has_map:
            entry["history"] = deque(maxlen=MIN_TRAIL_LEN if self.minimal else TRAIL_LEN)
            if self.minimal:
                entry.update(self._make_min_artists(self.ax_map, color, pose))
            else:
                entry.update(self._make_map_artists(self.ax_map, color, pose))
        if self.has_panel:
            entry.update({"hist": deque(),
                          "dirty": set(),       # échelles élargies depuis le dernier rafraîchissement du panneau
                          "lims": {"motor_lim": MOTOR_LIM, "thrust_lim": THRUST_LIM,
                                   "speed_max": SPEED_MAX, "speed_lo": 0.0}})
        self.artists[asset["id"]] = entry
        if self.selected_id is None:
            self.selected_id = asset["id"]
            if self.has_panel:
                self._build_panel()
        if self.has_panel:
            self._draw_radio()
        return entry

    @staticmethod
    def _hull_xy(pose: dict) -> np.ndarray:
        """Sommets du triangle (étrave vers le cap) représentant l'USV dans le repère monde."""
        L = MIN_HULL_LEN
        c, s = np.cos(pose["psi_rad"]), np.sin(pose["psi_rad"])
        pts = np.array([[L, 0.0], [-0.6 * L, 0.45 * L], [-0.6 * L, -0.45 * L]])
        rot = np.array([[c, -s], [s, c]])
        return pts @ rot.T + np.array([pose["x"], pose["y"]])

    def _make_min_artists(self, ax, color: str, pose: dict) -> dict:
        """Artistes minimaux d'un USV : un triangle (+ trace et trajet planifié en option).
        Tous sont enregistrés auprès du blitter (redessinés seuls à chaque tick)."""
        hull = Polygon(self._hull_xy(pose), closed=True, facecolor=color, edgecolor="white",
                       linewidth=1.0, zorder=7)
        ax.add_patch(hull)
        arts = {"hull": hull}
        if MIN_SHOW_TRAIL:
            arts["trail"], = ax.plot([], [], "-", color=color, lw=1.5, alpha=0.7, zorder=5)
        if MIN_SHOW_PATH:
            arts["path"], = ax.plot([], [], "--", color=color, lw=1.2, alpha=0.9, zorder=5)
            arts["target"], = ax.plot([], [], "*", color=TARGET, markersize=14,
                                      markeredgecolor="black", zorder=8)
        for a in arts.values():
            self.blitter.add(a)
        return arts
 
    def _make_map_artists(self, ax, color: str, pose: dict) -> dict:
        """Jeu d'artistes d'un USV sur la carte. Tous sont animés : redessinés seuls par le blitter
        à la cadence de la carte, sans toucher au reste de la fenêtre."""
        arrow = ax.annotate("", xy=(pose["x"], pose["y"]), xytext=_tail(pose), zorder=7,
                            arrowprops=dict(arrowstyle="-|>", color=color, lw=3, mutation_scale=22))
        est_arrow = ax.annotate("", xy=(pose["x"], pose["y"]), xytext=_tail(pose, EST_ARROW_LEN), zorder=9,
                                arrowprops=dict(arrowstyle="-|>", color="white", lw=1.4, mutation_scale=12))
        des_arrow = ax.annotate("", xy=(pose["x"], pose["y"]), xytext=(pose["x"], pose["y"]), zorder=9,
                                arrowprops=dict(arrowstyle="-|>", color=TARGET, lw=1.6, mutation_scale=14))
        des_arrow.set_visible(False)
        trail, = ax.plot([], [], "-", color=color, lw=1.8, alpha=0.7, zorder=5)
        path, = ax.plot([], [], "--", color=color, lw=1.5, alpha=0.9, zorder=5)
        estimate, = ax.plot([], [], "x", color="white", markersize=8, markeredgewidth=2, zorder=8)
        target, = ax.plot([], [], "*", color=TARGET, markersize=16, markeredgecolor="black", zorder=8)
        # annotation_clip=False : sans cela, matplotlib masque l'annotation dès que son point xy (0, 0,
        # jamais mis à jour) sort de la vue, ce qui arrive en vue de suivi zoomée.
        label = ax.annotate("", (0, 0), color=TARGET, fontsize=10, fontweight="bold", zorder=8,
                            annotation_clip=False, clip_on=True)
        arts = {"arrow": arrow, "est_arrow": est_arrow, "des_arrow": des_arrow, "trail": trail, "path": path,
                "estimate": estimate, "target": target, "target_label": label}
        for a in arts.values():
            self.blitter.add(a)
        return arts
 
    # ------------------------------------------------------- helpers de carte USV
    @staticmethod
    def _speed_ylim(lims: dict) -> tuple[float, float]:
        lo, hi = lims["speed_lo"], lims["speed_max"]
        span = hi - lo
        return lo - 0.08 * span, hi + 0.05 * span
 
    @staticmethod
    def _speed_bar_ylim(lims: dict) -> tuple[float, float]:
        hi = lims["speed_max"]
        return min(lims["speed_lo"], -0.25 * hi) * 1.1, hi * BAR_HEADROOM
 
    @staticmethod
    def _panel_style(ax, title: str, color: str, sc: float, text: bool = False):
        """Style commun d'un panneau de carte : fond, cadre de la couleur de l'USV, titre de rubrique.
        text=True : panneau de texte (axes 0-1 sans graduations). Renvoie l'objet titre."""
        ax.set_facecolor(BG_CARD)
        for s in ax.spines.values():
            s.set_color(color)
            s.set_linewidth(1.2)
        if text:
            ax.set_xlim(0, 1)
            ax.set_ylim(0, 1)
            ax.set_xticks([])
            ax.set_yticks([])
        return ax.set_title(title, loc="left", color=color, fontsize=9.5 * sc,
                            fontweight="bold", pad=4)
 
    @staticmethod
    def _make_rows(ax, rows: list, sc: float, fs: float = 11.5, meaning_x: float = 0.21) -> list:
        """Lignes « symbole | signification ........ valeur » réparties sur la hauteur du panneau.
        rows : liste de (symbole, signification) ; le symbole peut être du mathtext (entre signes dollar).
        Tout est aligné sur la même ligne de base. Renvoie les objets texte des valeurs."""
        n = len(rows)
        step = 0.92 / n
        values = []
        for i, (symbol, meaning) in enumerate(rows):
            top = 0.97 - i * step
            base = top - 0.68 * step
            if i:
                ax.plot([0.03, 0.97], [top, top], color=BORDER, lw=0.6, alpha=0.5, transform=ax.transAxes)
            ax.text(0.04, base, symbol, transform=ax.transAxes, color=TEXT, fontsize=(fs + 1) * sc,
                    va="baseline")
            ax.text(meaning_x, base, meaning, transform=ax.transAxes, color=TEXT_DIM, fontsize=(fs - 2.0) * sc,
                    va="baseline")
            values.append(ax.text(0.96, base, "", transform=ax.transAxes, color=TEXT, fontsize=fs * sc,
                                  ha="right", va="baseline", family="monospace", fontweight="bold"))
        return values
 
    def _make_bars(self, ax, title: str, labels: list, colors: list, ylim: tuple,
                   color: str, sc: float):
        """Histogramme instantané (une barre par grandeur, valeur écrite au bout de la barre)."""
        title_obj = self._panel_style(ax, title, color, sc)
        n = len(labels)
        ax.set_xlim(-0.6, n - 0.4)
        ax.set_ylim(*ylim)
        ax.set_axisbelow(True)
        ax.grid(True, axis="y", linestyle="--", alpha=0.25, color="white")
        bars = ax.bar(range(n), [0] * n, width=0.65, color=colors,
                      edgecolor="white", linewidth=0.8, zorder=3)
        ax.axhline(0, color=TEXT, lw=1, zorder=4)
        ax.set_xticks(range(n))
        ax.set_xticklabels(labels)
        ax.tick_params(colors=TEXT, labelsize=9 * sc)
        ax.yaxis.set_major_locator(MaxNLocator(nbins=5))
        texts = [ax.text(i, 0, "", ha="center", va="bottom", color=TEXT, fontsize=9.5 * sc,
                         family="monospace", zorder=5, clip_on=False) for i in range(n)]
        return list(bars.patches), texts, title_obj
 
    @staticmethod
    def _set_bars(bars: list, texts: list, values) -> None:
        for rect, txt, val in zip(bars, texts, values):
            if not np.isfinite(val):                    # valeur absente (ex. pas de consigne) : barre vide
                rect.set_height(0.0)
                txt.set_text("--")
                txt.set_position((rect.get_x() + rect.get_width() / 2.0, 0.0))
                txt.set_va("bottom")
                continue
            rect.set_height(val)
            txt.set_text(f"{val:+.2f}")
            txt.set_position((rect.get_x() + rect.get_width() / 2.0, val))
            txt.set_va("bottom" if val >= 0 else "top")
 
    def _make_history(self, ax, title: str, color: str, sc: float, ylim: tuple) -> None:
        """Style d'un graphe temporel glissant."""
        self._panel_style(ax, title, color, sc)
        ax.grid(True, linestyle="--", alpha=0.25, color="white")
        ax.set_axisbelow(True)
        ax.tick_params(colors=TEXT, labelsize=8.5 * sc, pad=2)
        ax.yaxis.set_major_locator(MaxNLocator(nbins=4))
        ax.set_ylim(*ylim)
 
    # ------------------------------------------------------------ corps du panneau
    def _make_body(self, color: str, lims: dict) -> dict:
        """Corps du panneau de l'USV sélectionné, de haut en bas :
          tableau de bord : ESTIMATED SPEED (bandeau pleine largeur : vitesse estimée + jauge)
                            MISSION STATUS  | ERRORS   (cible, consignes | erreurs de suivi et d'estimation)
                            GROUND TRUTH    | BELIEF   (x, y, ψ, u, v, r, β | x̂, ŷ, ψ̂, û, v̂, r̂, β̂)
          graphes         : [histogramme | historique 30 s] pour
                            MOTOR COMMANDS (Ul, Ur), THRUST (Tl, Tr), SPEEDS (u, û, u_des).
        Les axes sont créés par _new_ax pour pouvoir être détruits au changement d'USV."""
        sc = self.ui_scale
        dash = self.card[1].subgridspec(3, 2, height_ratios=[1.0, 1.75, 1.55], hspace=0.42, wspace=0.12)
        graphs = self.card[2].subgridspec(3, 2, width_ratios=[1.0, 2.6], hspace=0.5, wspace=0.18)
 
        # ---------------- Dashboard ----------------
        ax_v = self._new_ax(dash[0, :])        # vitesse estimée + jauge (bandeau pleine largeur)
        ax_n = self._new_ax(dash[1, 0])        # mission : cible, consignes
        ax_er = self._new_ax(dash[1, 1])       # erreurs : suivi + estimation
        ax_k = self._new_ax(dash[2, 0])        # vérité terrain (simulateur)
        ax_e = self._new_ax(dash[2, 1])        # croyance / estimation (brain)
        self._panel_style(ax_v, "ESTIMATED SPEED", color, sc, text=True)
        self._panel_style(ax_n, "MISSION STATUS", color, sc, text=True)
        self._panel_style(ax_er, "ERRORS · tracking / estimation", color, sc, text=True)
        self._panel_style(ax_k, "GROUND TRUTH · body", color, sc, text=True)
        self._panel_style(ax_e, "BELIEF · brain", color, sc, text=True)
 
        # -- Speed: value, gauge, setpoint (bandeau horizontal)
        tr = ax_v.transAxes
        speed = ax_v.text(0.03, 0.55, "", transform=tr, color=TEXT, fontsize=26 * sc, fontweight="bold",
                          family="monospace", va="center")
        speed_kn = ax_v.text(0.37, 0.55, "", transform=tr, color=TEXT_DIM, fontsize=13 * sc,
                             family="monospace", ha="right", va="center")
        gx, gy, gw, gh = self.GAUGE
        gauge_lbl = ax_v.text(gx, gy - 0.09, "", transform=tr, color=TEXT_DIM, fontsize=10.5 * sc,
                              family="monospace", va="top")
        gauge_scale = ax_v.text(gx + gw, gy - 0.09, "", transform=tr, color=TEXT_DIM, fontsize=10.5 * sc,
                                family="monospace", ha="right", va="top")
        ax_v.add_patch(Rectangle((gx, gy), gw, gh, transform=tr, facecolor=GAUGE_BG,
                                 edgecolor=BORDER, lw=0.8, zorder=2))
        gauge_fill = ax_v.add_patch(Rectangle((gx, gy), 0.0, gh, transform=tr,
                                              facecolor=color, edgecolor="none", zorder=3))
        gauge_sp, = ax_v.plot([gx, gx], [gy - 0.07, gy + gh + 0.07], color=TARGET, lw=2.5,
                              transform=tr, zorder=5, solid_capstyle="butt")
        gauge_sp.set_visible(False)
 
        # -- Text sections: (variable, meaning) -> value
        rows_n = self._make_rows(ax_n, [
            ("WP", "Waypoint (cur / total)"),
            (r"$x_t,\ y_t$", "Target position"),
            (r"$d_t$", "Distance to target"),
            (r"$\psi_{des}$", "Desired heading"),
            (r"$u_{des}$", "Desired speed"),
        ], sc)
        rows_er = self._make_rows(ax_er, [
            (r"$\psi_{des}-\hat{\psi}$", "Heading (tracking)"),
            (r"$u_{des}-\hat{u}$", "Speed (tracking)"),
            (r"$\Vert\hat{p}-p\Vert$", "Position (estim.)"),
            (r"$\hat{\psi}-\psi$", "Heading (estim.)"),
            (r"$\hat{u}-u$", "Surge speed (estim.)"),
            (r"$\hat{v}-v$", "Sway speed (estim.)"),
            (r"$\hat{r}-r$", "Yaw rate (estim.)"),
        ], sc, meaning_x=0.33)
        rows_k = self._make_rows(ax_k, [
            (r"$x,\ y$", "Position"),
            (r"$\psi$", "Heading"),
            (r"$u$", "Surge speed (forward)"),
            (r"$v$", "Sway speed (lateral)"),
            (r"$r$", "Yaw rate"),
            (r"$\beta$", "Drift angle"),
        ], sc)
        rows_e = self._make_rows(ax_e, [
            (r"$\hat{x},\ \hat{y}$", "Position"),
            (r"$\hat{\psi}$", "Heading"),
            (r"$\hat{u}$", "Surge speed (forward)"),
            (r"$\hat{v}$", "Sway speed (lateral)"),
            (r"$\hat{r}$", "Yaw rate"),
            (r"$\hat{\beta}$", "Drift angle"),
        ], sc)
 
        # ---------------- Graphs: histograms (left) ----------------
        ax_cm = self._new_ax(graphs[0, 0])
        ax_tm = self._new_ax(graphs[1, 0])
        ax_sm = self._new_ax(graphs[2, 0])
        ml, tl_ = lims["motor_lim"], lims["thrust_lim"]
 
        motor_bars, motor_texts, _ = self._make_bars(
            ax_cm, "MOTOR COMMANDS", ["L·Ul", "R·Ur"], [CMD_L, CMD_R],
            (-ml * BAR_HEADROOM, ml * BAR_HEADROOM), color, sc)
        thrust_bars, thrust_texts, thrust_title = self._make_bars(
            ax_tm, "THRUST", ["L·Tl", "R·Tr"], [THR_L, THR_R],
            (-tl_ * BAR_HEADROOM, tl_ * BAR_HEADROOM), color, sc)
        vel_bars, vel_texts, _ = self._make_bars(
            ax_sm, "SPEEDS (m/s)", [r"$u$", r"$\hat{u}$", r"$u_{des}$"], [color, TEXT, TARGET],
            self._speed_bar_ylim(lims), color, sc)
 
        # ---------------- Graphs: sliding histories (right) ----------------
        ax_c = self._new_ax(graphs[0, 1])
        ax_t = self._new_ax(graphs[1, 1], sharex=ax_c)
        ax_s = self._new_ax(graphs[2, 1], sharex=ax_c)
        self._make_history(ax_c, f"Motor commands Ul / Ur · {HISTORY_S:g} s", color, sc,
                           (-ml * 1.08, ml * 1.08))
        self._make_history(ax_t, f"Thrust Tl / Tr ({THRUST_UNIT}) · {HISTORY_S:g} s", color, sc,
                           (-tl_ * 1.08, tl_ * 1.08))
        self._make_history(ax_s, f"Speeds (m/s) · {HISTORY_S:g} s", color, sc,
                           self._speed_ylim(lims))
        ax_c.set_xlim(-HISTORY_S, 0)
        ax_c.set_xticks(np.linspace(-HISTORY_S, 0, 7))
        ax_c.tick_params(labelbottom=False)         # l'axe des temps n'est écrit que sur le graphe du bas
        ax_t.tick_params(labelbottom=False)
        ax_s.set_xlabel("time (s)", color=TEXT_DIM, fontsize=8.5 * sc, labelpad=1)
        for ax in (ax_c, ax_t):
            ax.axhline(0, color=TEXT, lw=0.8, alpha=0.6)
 
        line_ul, = ax_c.plot([], [], color=CMD_L, lw=1.4, label="Ul (L)")
        line_ur, = ax_c.plot([], [], color=CMD_R, lw=1.4, label="Ur (R)")
        line_tl, = ax_t.plot([], [], color=THR_L, lw=1.4, label="Tl (L)")
        line_tr, = ax_t.plot([], [], color=THR_R, lw=1.4, label="Tr (R)")
        line_u, = ax_s.plot([], [], color=color, lw=1.8, label=r"$u$ true")
        line_est, = ax_s.plot([], [], color=TEXT, lw=1.2, ls=":", label=r"$\hat{u}$ estimated")
        line_sp, = ax_s.plot([], [], color=TARGET, lw=1.3, ls="--", drawstyle="steps-post",
                             label=r"$u_{des}$ desired")
        for ax, ncol in ((ax_c, 2), (ax_t, 2), (ax_s, 3)):
            ax.legend(loc="upper left", ncol=ncol, fontsize=8.5 * sc, frameon=True, facecolor=BG_CARD,
                      edgecolor="none", framealpha=0.7, labelcolor=TEXT, borderaxespad=0.2,
                      handlelength=1.4, columnspacing=1.0)
 
        return {"dash_speed": speed, "dash_speed_kn": speed_kn,
                "gauge_fill": gauge_fill, "gauge_sp": gauge_sp,
                "gauge_lbl": gauge_lbl, "gauge_scale": gauge_scale,
                "rows_n": rows_n, "rows_er": rows_er, "rows_k": rows_k, "rows_e": rows_e,
                "ax_motor": ax_cm, "motor_bars": motor_bars, "motor_texts": motor_texts,
                "ax_thrust": ax_tm, "thrust_bars": thrust_bars, "thrust_texts": thrust_texts,
                "thrust_title": thrust_title,
                "ax_vbars": ax_sm, "vel_bars": vel_bars, "vel_texts": vel_texts,
                "ax_cmd": ax_c, "ax_thr": ax_t, "ax_speed": ax_s,
                "line_ul": line_ul, "line_ur": line_ur, "line_tl": line_tl, "line_tr": line_tr,
                "line_u": line_u, "line_est": line_est, "line_sp": line_sp}
 
    # ------------------------------------------------------------ vues
    def set_view(self, name: str):
        """Affiche la couche demandée (terrain SDF ou costmap) et met à jour les boutons."""
        self.current_view = name
        for key, layers in self.view_layers.items():
            for art in layers:
                art.set_visible(key == name)
        for key, btn in self.view_buttons.items():      # vide en mode minimal
            active = key == name
            btn.color = BTN_ON if active else BTN_OFF
            btn.hovercolor = BTN_ON if active else BTN_HOVER
            btn.ax.set_facecolor(btn.color)
        self.blitter.invalidate()                       # la carte statique a changé : fond à refaire
 
    # ------------------------------------------------------------ vue de suivi
    def set_follow(self, on: bool):
        """Active / désactive la vue centrée sur l'USV sélectionné (indépendante de la couche affichée).
        En suivi, la carte est un axe « animé » : seule elle est redessinée à chaque image, le reste de
        la fenêtre (boutons, légende, panneau en fenêtre unique) reste dans le fond en cache."""
        on = bool(on)
        if on == self.follow:
            return
        self.follow = on
        if on:
            self.follow_w = float(np.clip(FOLLOW_W0, ZOOM_MIN_W, self.home_w))
            self._apply_follow()
        else:                                           # retour à la vue d'ensemble
            self.ax_map.set_xlim(*self.home_xlim)
            self.ax_map.set_ylim(*self.home_ylim)
        self._refresh_follow_ui()
        self.blitter.set_axes_animated(self.ax_map, on)  # invalide aussi le fond
 
    def zoom(self, factor: float):
        """factor < 1 : zoom avant ; factor > 1 : zoom arrière. Sans effet hors suivi."""
        if not self.follow:
            return
        self.follow_w = float(np.clip(self.follow_w * factor, ZOOM_MIN_W, self.home_w))
        self._apply_follow()
        self._refresh_follow_ui()
        self.blitter.invalidate()
 
    def _apply_follow(self, snap=None):
        """Recentre ax_map sur l'USV sélectionné, en gardant le rapport d'aspect de la vue d'ensemble
        (sinon la boîte de la carte, en adjustable='box', se redimensionnerait)."""
        snap = snap or self.get_dynamic_snapshot()
        if snap is None:
            return
        pose = next((a["truth"]["pose"] for a in snap["assets"] if a["id"] == self.selected_id), None)
        if pose is None:
            return
        w = self.follow_w
        h = w * self.home_h / self.home_w
        self.ax_map.set_xlim(pose["x"] - w / 2, pose["x"] + w / 2)
        self.ax_map.set_ylim(pose["y"] - h / 2, pose["y"] + h / 2)
 
    def _refresh_follow_ui(self):
        """Couleur du bouton de suivi, état des boutons +/− et valeur du zoom."""
        if self.minimal:                                # pas de boutons en mode minimal
            return
        btn = self.follow_button
        btn.color = BTN_ON if self.follow else BTN_OFF
        btn.hovercolor = BTN_ON if self.follow else BTN_HOVER
        btn.ax.set_facecolor(btn.color)
        for b in self.zoom_buttons.values():
            b.label.set_color(TEXT if self.follow else TEXT_DIM)
        if self.follow:
            self.zoom_text.set_text(f"\u00d7{self.home_w / self.follow_w:.1f}\n{self.follow_w:.0f} m")
        else:
            self.zoom_text.set_text("zoom\n--")
        self.zoom_text.set_color(TEXT if self.follow else TEXT_DIM)
 
    def _on_key(self, event):
        if event.key == "tab" and self.selected_id is not None:     # raccourci : USV suivant (les 2 fenêtres)
            ids = list(self.artists)
            self.select_asset(ids[(ids.index(self.selected_id) + 1) % len(ids)])
        elif not self.has_map:                                      # fenêtre du panneau : pas d'autre raccourci
            return
        elif event.key == "m":                                      # raccourci : basculer terrain / costmap
            keys = list(self.VIEWS)
            self.set_view(keys[(keys.index(self.current_view) + 1) % len(keys)])
        elif event.key == "u":                                      # raccourci : suivi USV on/off
            self.set_follow(not self.follow)
        elif event.key in ("+", "="):                               # zoom avant (en suivi)
            self.zoom(1.0 / ZOOM_STEP)
        elif event.key in ("-", "_"):                               # zoom arrière (en suivi)
            self.zoom(ZOOM_STEP)
 
    # ------------------------------------------------------- plein écran
    def _schedule_fullscreen(self):
        """Passe en plein écran une fois la fenêtre réellement affichée."""
        try:
            self._fs_timer = self.fig.canvas.new_timer(interval=150)
            self._fs_timer.single_shot = True
            self._fs_timer.add_callback(self._go_fullscreen)
            self._fs_timer.start()
        except Exception:
            self._go_fullscreen()
 
    def _go_fullscreen(self):
        mng = self.fig.canvas.manager
        if mng is None:
            return
        backend = matplotlib.get_backend().lower()
        try:
            if "macosx" not in backend:        # macosx : pas d'API plein écran (bouton vert de la fenêtre)
                mng.full_screen_toggle()
            return
        except Exception:
            pass
        for maximize in (lambda: mng.window.showMaximized(),            # Qt
                         lambda: mng.window.state("zoomed"),            # Tk (Windows)
                         lambda: mng.window.attributes("-zoomed", True)):   # Tk (Linux)
            try:
                maximize()
                return
            except Exception:
                continue

    # ------------------------------------------------- processus du panneau (rôle "map")
    def _spawn_panel(self, snap):
        """Lance le processus du panneau (contexte « spawn » : fork est dangereux avec Qt) et lui envoie
        le premier snapshot, avec lequel il construit son interface."""
        ctx = mp.get_context("spawn")
        self._q_out = ctx.Queue(maxsize=IPC_QUEUE_MAX)      # carte -> panneau (snapshots, sélection, arrêt)
        self._q_in = ctx.Queue()                            # panneau -> carte (sélection)
        params = {"panel_fps": 1.0 / self.panel_dt}
        self._proc = ctx.Process(target=_panel_process_main, args=(self._q_out, self._q_in, params),
                                 daemon=True, name="usv-panel")
        self._proc.start()
        if snap is not None:
            self._send_snapshot(snap)

    def _send_snapshot(self, snap):
        """Envoie au panneau le snapshot courant, allégé. File pleine : le snapshot est abandonné
        (la carte ne doit jamais attendre le panneau)."""
        if self._proc is None:
            return
        if not self._proc.is_alive():
            if not self._panel_dead_reported:
                self._panel_dead_reported = True
                print(f"[GUI] processus du panneau arrêté (code {self._proc.exitcode}) : la carte continue seule.")
            return
        payload = {"time": snap["time"], "k": snap.get("k"),
                   "assets": [_light_asset(a) for a in snap["assets"]]}
        colors = {aid: e["color"] for aid, e in self.artists.items()}
        try:
            self._q_out.put_nowait(("snap", payload, colors))
        except queue.Full:
            pass

    def _poll_ipc(self):
        """(rôle "map") Lit les messages du panneau : sélection d'USV faite dans l'autre fenêtre."""
        if self._q_in is None:
            return
        while True:
            try:
                msg = self._q_in.get_nowait()
            except queue.Empty:
                break
            if msg[0] == "select":
                self.select_asset(msg[1], notify=False)

    def _shutdown_panel(self):
        proc = self._proc
        if proc is None:
            return
        try:
            if proc.is_alive():
                try:
                    self._q_out.put_nowait(("quit",))
                except Exception:
                    pass
                proc.join(timeout=2.0)
            if proc.is_alive():
                proc.terminate()
                proc.join(timeout=1.0)
        except Exception:
            pass
        for q in (self._q_out, self._q_in):
            try:
                q.cancel_join_thread()
                q.close()
            except Exception:
                pass

    # ------------------------------------------------- boucle du panneau (rôle "panel")
    def _panel_ingest(self, snap: dict, colors: dict):
        """Intègre un snapshot reçu : crée les nouveaux USV et alimente historiques et échelles
        (à CHAQUE tick reçu, pour que les courbes n'aient pas de trous)."""
        self._last_snap = snap
        for asset in snap["assets"]:
            entry = self.artists.get(asset["id"])
            if entry is None:
                entry = self.add_asset(asset, colors.get(asset["id"]))
            self._update_data(entry, asset, snap["time"])

    def _panel_refresh(self):
        """Met à jour les textes / barres / graphes du panneau d'après le dernier snapshot, puis redessine."""
        snap = self._last_snap
        if snap is None or not self.panel:
            return
        asset = next((a for a in snap["assets"] if a["id"] == self.selected_id), None)
        if asset is not None:
            self._update_panel(self.artists[self.selected_id], asset, snap["time"])
        self.fig.canvas.draw_idle()

    def _panel_poll(self):
        """Timer du panneau : vide la file (tous les snapshots reçus alimentent les historiques),
        puis ne redessine qu'une fois."""
        got = False
        while True:
            try:
                msg = self._q_in.get_nowait()
            except queue.Empty:
                break
            kind = msg[0]
            if kind == "snap":
                self._panel_ingest(msg[1], msg[2])
                got = True
            elif kind == "select":
                self.select_asset(msg[1], notify=False)
            elif kind == "quit":
                self.timer.stop()
                plt.close(self.fig)
                return
        if got:
            self._panel_refresh()
 
    # ----------------------------------------------------------- update
    def run(self):
        # Timer maison + blitting (FuncAnimation forcerait un rendu complet de la figure à chaque image).
        self.timer = self.fig.canvas.new_timer(
            interval=int((self.panel_dt if self.role == "panel" else self.dt) * 1000))
        self.timer.add_callback(self._panel_poll if self.role == "panel" else self.update)
        self.timer.start()
        if AUTO_PLACE_WINDOWS and self.role in ("map", "panel"):
            _place_window(self.fig, "left" if self.role == "map" else "right")
        elif FULLSCREEN and self.role in ("all", "min"):
            self._schedule_fullscreen()
        try:
            plt.show()
        finally:
            self._shutdown_panel()
 
    def update(self, _=None):
        if self.role == "map":
            self._poll_ipc()                        # sélection faite dans la fenêtre du panneau
        snap = self.get_dynamic_snapshot()
        if snap is None:
            return
        # Rendu et simulation ne sont pas synchronisés : on ne traite que les NOUVEAUX ticks,
        # sinon les traces et historiques recevraient des points dupliqués.
        k = snap.get("k")
        if k is not None:
            if k == self._last_k:
                return
            self._last_k = k

        # Fenêtre unique : le panneau n'est rafraîchi (et la figure entière redessinée) qu'à PANEL_FPS ;
        # entre deux, seuls les éléments animés de la carte sont redessinés (blitting).
        panel_due = False
        if self.role == "all":
            now = time.perf_counter()
            if now - self._last_panel_t >= self.panel_dt:
                self._last_panel_t = now
                panel_due = True

        t_now = snap["time"]
        self.clock.set_text(f"t = {t_now:.1f} s")
        for asset in snap["assets"]:
            entry = self.artists.get(asset["id"])
            if entry is None:                       # nouvel USV arrivé en cours de simulation
                entry = self.add_asset(asset)
            if self.minimal:
                self.update_asset_min(entry, asset)
                continue
            self._update_map_artists(entry, asset)
            if self.has_panel:                      # rôle "all" : données à chaque tick, panneau à PANEL_FPS
                self._update_data(entry, asset, t_now)
                if panel_due and asset["id"] == self.selected_id and self.panel:
                    self._update_panel(entry, asset, t_now)
        if self.follow:                             # vue de suivi : recentrage à chaque image
            self._apply_follow(snap)

        if self.role == "map":
            self._send_snapshot(snap)               # le panneau (autre processus) fait le reste

        if panel_due:
            self.fig.canvas.draw_idle()             # rendu complet : panneau à jour, fond recapturé
        else:
            self.blitter.update()                   # redessin partiel : carte animée + horloge

    def update_asset_min(self, entry: dict, asset: dict):
        """Mise à jour minimale d'un USV : position / cap du triangle (+ trace et trajet si activés)."""
        pose = asset["truth"]["pose"]
        entry["hull"].set_xy(self._hull_xy(pose))
        if "trail" in entry:
            entry["history"].append((pose["x"], pose["y"]))
            trail_x, trail_y = zip(*entry["history"])
            entry["trail"].set_data(trail_x, trail_y)
        if "path" in entry:
            belief = asset["belief"]
            path = belief["path"]
            tgt = belief["ongoing_target"]
            entry["path"].set_data([p["x"] for p in path], [p["y"] for p in path])
            if tgt is None:
                entry["target"].set_data([], [])
            else:
                entry["target"].set_data([tgt["x"]], [tgt["y"]])

    def _update_map_artists(self, entry: dict, asset: dict):
        """Met à jour, pour TOUS les USV et à chaque tick, les artistes de la carte (animés :
        redessinés seuls par le blitter)."""
        d = _unpack(asset)
        pose, est, tgt, psi_des = d.pose, d.est, d.tgt, d.psi_des
        path = d.path or []

        entry["history"].append((pose["x"], pose["y"]))
        trail_x, trail_y = zip(*entry["history"])

        entry["arrow"].set_position(_tail(pose))
        entry["arrow"].xy = (pose["x"], pose["y"])
        entry["est_arrow"].set_position(_tail(est, EST_ARROW_LEN))
        entry["est_arrow"].xy = (est["x"], est["y"])
        if psi_des is None:
            entry["des_arrow"].set_visible(False)
        else:
            entry["des_arrow"].set_position((est["x"], est["y"]))
            entry["des_arrow"].xy = (est["x"] + np.cos(psi_des) * DES_ARROW_LEN,
                                     est["y"] + np.sin(psi_des) * DES_ARROW_LEN)
            entry["des_arrow"].set_visible(True)
        entry["trail"].set_data(trail_x, trail_y)
        entry["estimate"].set_data([est["x"]], [est["y"]])
        entry["path"].set_data([p["x"] for p in path], [p["y"] for p in path])
        if tgt is None:
            entry["target"].set_data([], [])
            entry["target_label"].set_visible(False)
        else:
            entry["target"].set_data([tgt["x"]], [tgt["y"]])
            entry["target_label"].set_text(f'TARGET {asset["id"]}')
            entry["target_label"].set_position((tgt["x"], tgt["y"] - 3))
            entry["target_label"].set_visible(True)

    def _update_data(self, entry: dict, asset: dict, t_now: float):
        """Données du panneau (échelles qui s'élargissent, historique glissant), alimentées à CHAQUE tick
        pour tous les USV, même quand le panneau n'est pas redessiné."""
        d = _unpack(asset)
        lims = entry["lims"]
        changed = set()
        peak = max(abs(d.ul), abs(d.ur))
        if peak > lims["motor_lim"]:
            lims["motor_lim"] = _nice(peak)
            changed.add("motor")
        peak = max(abs(d.tl), abs(d.tr))
        if peak > lims["thrust_lim"]:
            lims["thrust_lim"] = _nice(peak)
            changed.add("thrust")
        need = max(d.u, d.u_hat, d.sp if d.sp is not None else 0.0)
        if need > lims["speed_max"]:
            lims["speed_max"] = _nice(need)
            changed.add("speed")
        low = min(d.u, d.u_hat)
        if low < lims["speed_lo"]:
            lims["speed_lo"] = -_nice(-low)
            changed.add("speed")
        entry["dirty"] |= changed                       # cumulé jusqu'au prochain rafraîchissement du panneau

        h = entry["hist"]
        if h and t_now < h[-1][0]:                      # simulation relancée : on repart de zéro
            h.clear()
        h.append((t_now, d.ul, d.ur, d.tl, d.tr, d.u, d.u_hat, np.nan if d.sp is None else d.sp))
        while h[0][0] < t_now - HISTORY_S:
            h.popleft()
 
    def _update_panel(self, entry: dict, asset: dict, t_now: float):
        """Rafraîchit textes, barres et graphes du panneau pour l'USV sélectionné."""
        d = _unpack(asset)
        belief = d.belief
        pose, est, tgt, sp, psi_des = d.pose, d.est, d.tgt, d.sp, d.psi_des
        u, v, r, u_hat, v_hat, r_hat = d.u, d.v, d.r, d.u_hat, d.v_hat, d.r_hat
        ul, ur, tl, tr = d.ul, d.ur, d.tl, d.tr
        lims = entry["lims"]
        h = entry["hist"]
        changed = entry["dirty"]
        entry["dirty"] = set()
        p = self.panel
 
        # ---- échelles du panneau ----
        if "motor" in changed:
            lim = lims["motor_lim"]
            p["ax_motor"].set_ylim(-lim * BAR_HEADROOM, lim * BAR_HEADROOM)
            p["ax_cmd"].set_ylim(-lim * 1.08, lim * 1.08)
        if "thrust" in changed:
            lim = lims["thrust_lim"]
            p["ax_thrust"].set_ylim(-lim * BAR_HEADROOM, lim * BAR_HEADROOM)
            p["ax_thr"].set_ylim(-lim * 1.08, lim * 1.08)
        if "speed" in changed:
            p["ax_speed"].set_ylim(*self._speed_ylim(lims))
            p["ax_vbars"].set_ylim(*self._speed_bar_ylim(lims))
 
        # ---- ESTIMATED SPEED : valeur estimée + jauge avec consigne ----
        vmax = lims["speed_max"]
        gx, _, gw, _ = self.GAUGE
        p["dash_speed"].set_text(f"{u_hat:.2f} m/s")
        p["dash_speed_kn"].set_text(f"{u_hat * MS_TO_KN:.1f} kn")
        p["gauge_fill"].set_width(gw * float(np.clip(u_hat / vmax, 0.0, 1.0)))
        p["gauge_scale"].set_text(f"0-{vmax:g} m/s")
        if sp is None:
            p["gauge_sp"].set_visible(False)
            p["gauge_lbl"].set_text("no setpoint")
            p["gauge_lbl"].set_color(TEXT_DIM)
            speed_err = None
        else:
            x_sp = gx + gw * float(np.clip(sp / vmax, 0.0, 1.0))
            p["gauge_sp"].set_xdata([x_sp, x_sp])
            p["gauge_sp"].set_visible(True)
            speed_err = sp - u_hat                              # erreur de suivi : consigne - estimée
            p["gauge_lbl"].set_text(f"u_des {sp:.2f}  u_des-û {speed_err:+.2f}")
            p["gauge_lbl"].set_color(OK_COLOR if abs(speed_err) <= SPEED_TOL else WARN_COLOR)
 
        # ---- MISSION STATUS : cible, consignes ----
        rn = p["rows_n"]
        rn[0].set_text(f'{belief["path_index"]}/{d.n_path}' if d.n_path else "--")
        if tgt is None:
            rn[1].set_text("--")
            rn[2].set_text("--")
        else:
            rn[1].set_text(f'{tgt["x"]:.1f}, {tgt["y"]:.1f} m')
            rn[2].set_text(f'{np.hypot(tgt["x"] - est["x"], tgt["y"] - est["y"]):.1f} m')
        rn[3].set_text("--" if psi_des is None else f"{np.degrees(psi_des) % 360.0:.1f} °")
        rn[4].set_text("--" if sp is None else f"{sp:.2f} m/s")
 
        # ---- ERRORS : suivi (consigne - estimé) puis estimation (estimé - vrai) ----
        rr = p["rows_er"]
        if psi_des is None:
            rr[0].set_text("--")
            rr[0].set_color(TEXT)
        else:
            herr = _wrap_deg(psi_des - est["psi_rad"])
            rr[0].set_text(f"{herr:+.1f} °")
            rr[0].set_color(OK_COLOR if abs(herr) <= HEADING_TOL_DEG else WARN_COLOR)
        if speed_err is None:
            rr[1].set_text("--")
            rr[1].set_color(TEXT)
        else:
            rr[1].set_text(f"{speed_err:+.2f} m/s")
            rr[1].set_color(OK_COLOR if abs(speed_err) <= SPEED_TOL else WARN_COLOR)
        rr[2].set_text(f'{np.hypot(pose["x"] - est["x"], pose["y"] - est["y"]):.2f} m')
        rr[3].set_text(f'{_wrap_deg(est["psi_rad"] - pose["psi_rad"]):+.1f} °')
        rr[4].set_text(_fmt(u_hat - u, "+.2f", " m/s"))
        rr[5].set_text(_fmt(v_hat - v, "+.2f", " m/s"))
        rr[6].set_text(_fmt(np.degrees(r_hat - r), "+.1f", " °/s"))
 
        # ---- GROUND TRUTH (simulateur) ----
        rk = p["rows_k"]
        rk[0].set_text(f'{pose["x"]:.1f}, {pose["y"]:.1f} m')
        rk[1].set_text(f'{np.degrees(pose["psi_rad"]) % 360.0:.1f} °')
        rk[2].set_text(f"{u:+.2f} m/s")
        rk[3].set_text(f"{v:+.2f} m/s")
        rk[4].set_text(f"{np.degrees(r):+.1f} °/s")
        rk[5].set_text(f"{_drift_deg(u, v):+.1f} °")
 
        # ---- BELIEF (brain) : mêmes grandeurs que la vérité terrain, version estimée ----
        re = p["rows_e"]
        re[0].set_text(f'{est["x"]:.1f}, {est["y"]:.1f} m')
        re[1].set_text(f'{np.degrees(est["psi_rad"]) % 360.0:.1f} °')
        re[2].set_text(_fmt(u_hat, "+.2f", " m/s"))
        re[3].set_text(_fmt(v_hat, "+.2f", " m/s"))
        re[4].set_text(_fmt(np.degrees(r_hat), "+.1f", " °/s"))
        re[5].set_text(f"{_drift_deg(u_hat, v_hat):+.1f} °")
 
        # ---- histogrammes ----
        self._set_bars(p["motor_bars"], p["motor_texts"], (ul, ur))
        self._set_bars(p["thrust_bars"], p["thrust_texts"], (tl, tr))
        self._set_bars(p["vel_bars"], p["vel_texts"], (u, u_hat, np.nan if sp is None else sp))
        p["thrust_title"].set_text(f"THRUST  Σ {tl + tr:+.2f} {THRUST_UNIT}")
 
        # ---- historiques glissants ----
        a = np.asarray(h, dtype=float)
        x = a[:, 0] - t_now                             # 0 = maintenant, négatif = passé
        p["line_ul"].set_data(x, a[:, 1])
        p["line_ur"].set_data(x, a[:, 2])
        p["line_tl"].set_data(x, a[:, 3])
        p["line_tr"].set_data(x, a[:, 4])
        p["line_u"].set_data(x, a[:, 5])
        p["line_est"].set_data(x, a[:, 6])
        p["line_sp"].set_data(x, a[:, 7])


def _panel_process_main(q_in, q_out, params: dict):
    """Point d'entrée du processus du panneau (doit rester au niveau module pour être importable par
    « spawn »). Attend le premier snapshot du processus principal, construit la fenêtre du panneau,
    puis tourne sur sa propre boucle Qt, à sa propre cadence."""
    msg = q_in.get(timeout=120)                         # ("snap", snapshot, couleurs)
    snap, colors = msg[1], msg[2]
    gui = graphical_interface(params, lambda: {}, lambda: snap,
                              _role="panel", _ipc=(q_in, q_out), _colors=colors)
    gui._panel_ingest(snap, colors)
    gui._panel_refresh()
    gui.run()