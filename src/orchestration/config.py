import numpy as np

# Reference point for lat/lon to x y conversion (currently Paris, can be changed to any other location)
LAT0 = 48.8566
LON0 = 2.3522
METER_PER_DEG_LAT = 111320.0
METER_PER_DEG_LON = 111320.0 * np.cos(np.radians(LAT0))

###
# The current config is a USV that rese;bles the specs of the Turkish USV Arida-M Counter-USV
###

# ───────────── USV specs: inputs only ─────────────
USV_inputs = {
    # general specs
    "m": 1000,        # kg, loaded displacement
    "length": 5,          # m
    "width": 1.3,         # m
    "u_max": 27,          # m/s (~97 km/h), performance target used to derive dq_u

    # propulsion
    "T_max": 1000,        # N per motor, static thrust
    "tau_motors": 1,      # s, motor modeled as first order wrt cmd
    "prop_lever": 0.5,    # m, lever arm from center of gravity to each motor

    # added mass, as a fraction of m and Iz
    "added_u_ratio": 0.10,
    "added_v_ratio": 0.80,
    "added_r_ratio": 1.00,

    # surge drag (dq_u is derived below)
    "dl_u": 30,           # N·s/m

    # sway drag
    "dl_v": 1000,
    "dq_v": 2800,

    # yaw drag
    "dl_r": 800,
    "dq_r": 562,
}


# ───────────── USV specs: derived values (never edit by hand) ─────────────
def derive_USV_specs(specs: dict) -> dict:
    s = dict(specs)

    s["Iz"] = s["m"] * (s["length"]**2 + s["width"]**2) / 12 #Rectangle shape approximation
    s["m_u"] = s["m"] * (1 + s["added_u_ratio"])
    s["m_v"] = s["m"] * (1 + s["added_v_ratio"])
    s["m_r"] = s["Iz"] * (1 + s["added_r_ratio"])

    s["dq_u"] = (2 * s["T_max"] - s["dl_u"] * s["u_max"]) / s["u_max"]**2

    return s


parameters = {
    "dt": 0.05,
    "dx": 0.1,
    "dy": 0.1,
    "nb_of_USVs": 1,
    "USV_specs": derive_USV_specs(USV_inputs),
}