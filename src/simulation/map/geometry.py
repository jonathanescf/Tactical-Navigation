import numpy as np

def circle(cx, cy, r):
    return lambda x, y: np.hypot(x - cx, y - cy) - r

def rectangle(p0, p1, p2, p3):
    """
    SDF d'un rectangle (éventuellement tourné) à partir de ses 4 sommets.
    Les sommets doivent être donnés dans l'ordre, en faisant le tour de la forme
    (horaire ou antihoraire) : p0 -> p1 -> p2 -> p3.
    """
    p0, p1, p2, p3 = (np.asarray(p, dtype=float) for p in (p0, p1, p2, p3))

    center = (p0 + p1 + p2 + p3) / 4
    e1, e2 = p1 - p0, p3 - p0                  # deux côtés adjacents
    h1, h2 = np.linalg.norm(e1) / 2, np.linalg.norm(e2) / 2   # demi-dimensions
    u, v = e1 / (2 * h1), e2 / (2 * h2)        # axes unitaires du rectangle

    if abs(np.dot(u, v)) > 1e-6:
        raise ValueError("Les sommets ne forment pas un rectangle")

    def f(x, y):
        dx, dy = x - center[0], y - center[1]
        a = dx * u[0] + dy * u[1]              # coordonnées dans le repère du rectangle
        b = dx * v[0] + dy * v[1]
        qa = np.abs(a) - h1                    # > 0 : au-delà du côté
        qb = np.abs(b) - h2
        outside = np.hypot(np.maximum(qa, 0), np.maximum(qb, 0))
        inside = np.minimum(np.maximum(qa, qb), 0)
        return outside + inside

    return f

def polygon(points):
    """SDF exacte d'un polygone simple : négative dedans, positive dehors."""
    A = np.asarray(points, dtype=float)
    B = np.roll(A, -1, axis=0)                       # sommet suivant de chaque arête
    ex, ey = B[:, 0] - A[:, 0], B[:, 1] - A[:, 1]

    def f(x, y):
        px = np.asarray(x, dtype=float)[..., None]   # axe supplémentaire = arêtes
        py = np.asarray(y, dtype=float)[..., None]
        wx, wy = px - A[:, 0], py - A[:, 1]
        t = np.clip((wx * ex + wy * ey) / (ex**2 + ey**2), 0, 1)
        d = np.hypot(wx - t * ex, wy - t * ey).min(axis=-1)      # distance à l'arête la plus proche

        straddle = (A[:, 1] <= py) != (B[:, 1] <= py)            # l'arête coupe la ligne horizontale
        x_cross = A[:, 0] + (py - A[:, 1]) * ex / np.where(ey == 0, 1, ey)
        inside = (straddle & (px < x_cross)).sum(axis=-1) % 2 == 1   # parité des croisements
        return np.where(inside, -d, d)

    return f

def union(*fields):
    def f(x, y):
        d = fields[0](x, y)
        for g in fields[1:]:
            d = np.minimum(d, g(x, y))   # un seul tableau en mémoire
        return d
    return f
