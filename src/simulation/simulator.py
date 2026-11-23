from time import time
import numpy as np


class simulator:
    def __init__(self, terrain, bodies, parameters):
        self.terrain = terrain
        self.bodies = bodies
        self.t = 0.0
        self.dt = parameters["dt"]

        self.m = parameters["USV_specs"]["m"]
        self.length = parameters["USV_specs"]["length"]
        self.prop_lever = parameters["USV_specs"]["prop_lever"]
        self.added_u_ratio = parameters["USV_specs"]["added_u_ratio"]
        self.added_v_ratio = parameters["USV_specs"]["added_v_ratio"]
        self.added_r_ratio = parameters["USV_specs"]["added_r_ratio"]
        self.dl_u = parameters["USV_specs"]["dl_u"]
        self.dq_u = parameters["USV_specs"]["dq_u"]
        self.dl_v = parameters["USV_specs"]["dl_v"]
        self.dq_v = parameters["USV_specs"]["dq_v"]
        self.dl_r = parameters["USV_specs"]["dl_r"]
        self.dq_r = parameters["USV_specs"]["dq_r"]
        self.u_max = parameters["USV_specs"]["u_max"]
        self.T_max = parameters["USV_specs"]["T_max"]
        self.Iz  = parameters["USV_specs"]["Iz"]   
        self.tau_motors= 2 # in sec. rappel 3 * tau ~= 95 percent of final value

        # Mass/Inertia matrix ussing added mass (and Inertia)
               
        self.m_u = self.m * (1 + self.added_u_ratio)     
        self.m_v = self.m * (1 + self.added_v_ratio)         
        self.m_r = self.Iz * (1 + self.added_r_ratio)        
        self.M = np.diag([self.m_u, self.m_v, self.m_r])
        self.M_inv = np.linalg.inv(self.M)


    def USV_physic_step(self, body, dt):

        if body.terrain.distance(body.x, body.y) < 0:
            print("WARNING: COLLISION WITH AN OBSTACLE")
            return

        # Thrust matrix at each dt the new (Tl, Tr) from (Ul, Ur)
        Tl_cmd = self.T_max * body.Ul / 255
        Tr_cmd = self.T_max * body.Ur / 255
        body.Tl = body.Tl + (Tl_cmd - body.Tl) * dt / self.tau_motors
        body.Tr = body.Tr + (Tr_cmd - body.Tr) * dt / self.tau_motors
        
        X_prop = body.Tl + body.Tr
        Y_prop = 0
        N_prop = self.prop_lever * (body.Tr - body.Tl)
        
        T_prop = np.array([X_prop,
                           Y_prop,
                           N_prop])

        # Drag matrix
        X_hydro = - self.dl_u * body.u - self.dq_u * np.abs(body.u) * body.u
        Y_hydro = - self.dl_v * body.v - self.dq_v * np.abs(body.v) * body.v
        N_hydro = - self.dl_r * body.r - self.dq_r * np.abs(body.r) * body.r

        T_hydro = np.array([X_hydro,
                            Y_hydro,
                            N_hydro])

        # Coriolis matrix
        C = np.array([[0                  , 0                  , - self.m_v * body.v],
                      [0                  , 0                  , + self.m_u * body.u],
                      [+ self.m_v * body.v, - self.m_u * body.u, 0                  ]])

        # Bilan des forces exterieurs
        T_tot = T_prop + T_hydro

        # updating nu using euler and Fossen formula in 3ddl config
        nu_dot = self.M_inv @ (T_tot - C @ body.nu)
        body.nu += nu_dot * dt

        # updating eta using euler and the cinematic relation between eta and nu
        c, s = np.cos(body.psi_rad), np.sin(body.psi_rad)
        Rot_mat = np.array([[c, -s, 0],
                            [s, c, 0],
                            [0, 0, 1]])

        eta_dot = Rot_mat @ body.nu
        body.eta += eta_dot * dt


    def USV__obsolete_physic_step(self, body, dt:float): # old and too simple
        """
        Used to update the USV state using physics equations.
        K and L are arbitrary, further research should be conducted to make the physics more realistic
        """

        with body.usv_lock:
            # modèle idéal : vitesse instantanée
            K = body.max_speed / 255
            L = 1
            v     =  K * (body.Ul + body.Ur) / 2
            omega = K * (body.Ur - body.Ul) / (2 * L)

            x_new = body.x + v * np.cos(body.psi_rad) * dt
            y_new = body.y + v * np.sin(body.psi_rad) * dt

            if body.terrain.distance(x_new, y_new) < 0:
                print("WARNING: COLLISION WITH AN OBSTACLE")

            else:
                body.x = x_new
                body.y = y_new
                body.psi_rad  = (body.psi_rad + omega * dt) % (2 * np.pi)


    def global_physic_step(self):
        for body in self.bodies.values():
            self.USV_physic_step(body, self.dt)
            print(f"x={body.x:8.2f} y={body.y:8.2f} psi={np.degrees(body.psi_rad):7.2f}° | "
                  f"u={body.u:6.3f} v={body.v:6.3f} r={body.r:6.3f} | "
                  f"Ul={body.Ul:7.1f} Ur={body.Ur:7.1f} Tl={body.Tl:7.1f} Tr={body.Tr:7.1f}")
        self.t += self.dt