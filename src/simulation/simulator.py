import numpy as np


class simulator:
    def __init__(self, bodies, parameters):
        self.bodies = bodies
        self.t = 0.0
        self.dt = parameters["dt"]

        self.m = parameters["USV_specs"]["m"]
        self.length = parameters["USV_specs"]["length"]
        self.prop_lever = parameters["USV_specs"]["prop_lever"]
        self.added_u_ratio = parameters["USV_specs"]["added_u_ratio"]
        self.added_v_ratio = parameters["USV_specs"]["added_v_ratio"]
        self.added_r_ratio = parameters["USV_specs"]["added_r_ratio"]
        self.m_u = parameters["USV_specs"]["m_u"]    
        self.m_v = parameters["USV_specs"]["m_v"]        
        self.m_r = parameters["USV_specs"]["m_r"]
        self.dl_u = parameters["USV_specs"]["dl_u"]
        self.dq_u = parameters["USV_specs"]["dq_u"]
        self.dl_v = parameters["USV_specs"]["dl_v"]
        self.dq_v = parameters["USV_specs"]["dq_v"]
        self.dl_r = parameters["USV_specs"]["dl_r"]
        self.dq_r = parameters["USV_specs"]["dq_r"]
        self.u_max = parameters["USV_specs"]["u_max"]
        self.T_max = parameters["USV_specs"]["T_max"]
        self.Iz  = parameters["USV_specs"]["Iz"]   
        self.tau_motors= parameters["USV_specs"]["tau_motors"] # in sec. rappel 3 * tau ~= 95 percent of final value

        # Mass/Inertia matrix ussing added mass (and Inertia)      
        self.M = np.diag([self.m_u, self.m_v, self.m_r])
        self.M_inv = np.linalg.inv(self.M)

    def USV_physic_step(self, body, dt):

        if body.terrain.distance(body.x, body.y) < 0:
            print("WARNING: COLLISION WITH AN OBSTACLE")
            return

        # Thrust matrix at each dt the new (Tl, Tr) from (Ul, Ur)
        if body.Ul > 0:
            Tl_cmd = self.T_max * body.Ul / 255
        else: 
            Tl_cmd = self.T_max/2 * body.Ul / 255

        if body.Ur > 0:
                    Tr_cmd = self.T_max * body.Ur / 255
        else: 
            Tr_cmd = self.T_max/2 * body.Ur / 255
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

    def USV_obsolete_physic_step(self, body, dt:float): # old and too simple
        """
        Used to update the USV state using physics equations.
        K and L are arbitrary, further research should be conducted to make the physics more realistic
        """

        with body.usv_lock:
            # modèle idéal : vitesse instantanée
            K = body.max_speed / 255
            L = 1
            u     =  K * (body.Ul + body.Ur) / 2
            omega = K * (body.Ur - body.Ul) / (2 * L)

            x_new = body.x + u * np.cos(body.psi_rad) * dt
            y_new = body.y + u * np.sin(body.psi_rad) * dt
            psi_rad_new = (body.psi_rad + omega * dt) % (2 * np.pi)

            eta_new = np.array([x_new, y_new, psi_rad_new])

            if body.terrain.distance(x_new, y_new) < 0:
                print("WARNING: COLLISION WITH AN OBSTACLE")

            else:
                body.eta = eta_new

    def global_physic_step(self):
        for body in self.bodies.values():
            self.USV_obsolete_physic_step(body, self.dt)
        self.t += self.dt