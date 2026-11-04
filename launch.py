import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))

# Un seul endroit à modifier quand un fichier bouge.
SIM_CMD  = [sys.executable, "main.py"]
PLOT_CMD = [sys.executable, "-m", "tools.plot"]


def screen_size():
    """Taille de l'écran (ou de l'ensemble des écrans côte à côte)."""
    try:
        import tkinter as tk
        root = tk.Tk()
        root.withdraw()
        size = root.winfo_screenwidth(), root.winfo_screenheight()
        root.destroy()
        return size
    except Exception:
        return 1920, 1080            # valeur de repli si Tk est indisponible


def start(cmd, geometry):
    """Lance un processus en lui passant x, y, largeur, hauteur de sa fenêtre."""
    env = {**os.environ, "WINDOW_GEOMETRY": ",".join(map(str, geometry))}
    return subprocess.Popen(cmd, env=env, cwd=ROOT)


if __name__ == "__main__":
    W, H = screen_size()
    half, h = W // 2, H - 80         # 80 px de marge pour la barre de titre et le dock

    procs = [
        start(SIM_CMD,  (0,    0, half, h)),
        start(PLOT_CMD, (half, 0, half, h)),
    ]
    try:
        for p in procs:
            p.wait()
    except KeyboardInterrupt:
        pass
    finally:
        for p in procs:
            if p.poll() is None:
                p.terminate()