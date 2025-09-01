import numpy as np
import cauchy_estimator as ce 
import gaussian_filters as gf
import matplotlib.pyplot as plt
# from pendulum_device import C2000_Communication as C2000
import time


class PendulumParams:
    """
    Steps:
    ----
    0. Edit the dynamics in dynamic_update, nonlinear_msmt_model, msmt_model_jacobian, pend_ode, jacobian_pendulum_ode, nonlin_transition_model
    1. Initialize the object with sampling time and number of control
    2. Set initial value x0, P0
    3. Set Cauchy filter parameters A0, b0 (normally you don't)
    4. Set Cauchy debug options
    5. Start Cauchy filter
    6. Propagate
    7. Shutdown Cauchy filter
    """
    def __init__(self, dt=0.001, num_controls=0):
        self.system_type = "nonlin"
        self.num_states = 2

        # Pendulum parameters
        self.g = 9.81
        self.B = 8.1055e-6
        self.L = .23e-3
        self.R = 3.85
        self.V_s = 10.7
        self.K_m = 0.0228
        self.m = 0.03937
        self.l_c = 0.0254
        self.J_motor = 1.67e-6
        self.J_rod = 2.12e-5
        self.EncRes = 400
        
        # Sampling parameters
        self.dt = dt
        self.sr = 1/self.dt

        # Induced parameters
        self.J = self.J_motor + self.J_rod
        self.Enc_n = 2 * np.pi / self.EncRes
        self.VelRes = self.EncRes / self.dt

        # matrices
        self.H = np.array([[1.0,0.0]])              # measurement matrix
        self.Gamma_c = np.array([[0.0],[1.0]])      # process noise control matrix
        self.x0 = np.zeros([self.num_states,1])
        self.P0 = np.zeros([self.num_states,self.num_states])

        # ---- Measurement noise setting
        # uniform noise [-a,a] -> beta: a/beta = 2.33
        # a / beta ratio = 2.3305
        self.scale_u2c = 1/3**.5
        self.gamma = np.array([self.Enc_n * self.scale_u2c])

        # uniform noise [-a,a] -> sigma
        self.scale_u2g = 1/np.sqrt(-2*np.log(2**.5 / 4))
        self.v_PSD = np.array((self.Enc_n*self.scale_u2g)**2)

        # ---- Process noise setting
        self.scale_g2c = 1.0/1.3898
        self.scale_c2g = 1.3898
        
        self.w_PSD = 15**2
        self.beta = np.array([(self.w_PSD/self.dt)**.5 * self.scale_g2c])

        self.w_PSD = np.array([[self.w_PSD]])
        self.v_PSD = np.array([[self.v_PSD]])

        # Assistive parameters
        self.idty = np.eye(self.num_states)
        
        # Controller gain
        self.num_controls = num_controls
        self.K_gain = np.zeros([self.num_controls,self.num_states])

        # Kalman filter variables
        self.x_kf = np.zeros([self.num_states, 1])
        self.P_kf = np.zeros([self.num_states, self.num_states])

        self.xs_kf = []
        self.Ps_kf = []
        
        # Cauchy filter setting
        self.A0 = np.eye(self.num_states)
        self.p0 = np.diag(np.eye(self.num_states)*.003)**0.5 * self.scale_g2c 
        self.b0 = np.zeros(self.num_states)

        # Cauchy filter debug setting
        self.print_debug = True
        self.swm_print_debug = False 
        self.win_print_debug = False

    def set_K_gain(self, K):
        self.K_gain = K

    def set_A0(self, A0):
        self.A0 = A0
    def set_b0(self, b0):
        self.b0 = b0
    
    def set_num_states(self, num_states):
        self.num_states = num_states

    def set_initial(self, x0, P0):
        self.x0 = x0.copy()
        self.x0_kf = np.random.multivariate_normal(x0.reshape(-1), P0).reshape([self.num_states,1])
        self.x_kf = self.x0_kf.reshape(-1)
        self.x0_cf = self.x0_kf.copy()

        self.P0 = P0.copy()
        self.P0_kf = P0.copy()
        self.P_kf = P0.copy()
        self.p0 = np.diag(self.P0_kf)**0.5 * self.scale_g2c

        self.xs_kf.clear()
        self.Ps_kf.clear()
        
        self.xs_kf = [self.x_kf.copy()]
        self.Ps_kf = [self.P_kf.copy()]

    # ----- Define system dynamics -----
    # The ODE
    def pend_ode(self, x, u=None):
        dx_dt = np.zeros(2)
        dx_dt[0] = x[1]
        # dx_dt[1] = -mp.g / mp.L * np.sin(x[0]) - mp.c * x[1]
        dx_dt[1] = -(self.B/self.J + self.K_m**2/(self.J*self.R))*x[1]-self.m*self.g*self.l_c*np.sin(x[0])/self.J
        if u is not None:
            dx_dt[1] += self.K_m * self.V_s / self.R/self.J * u
        return dx_dt

    # Jacobian
    def jacobian_pendulum_ode(self, x):
        Jac = np.zeros((2,2))
        Jac[0,1] = 1
        Jac[1,0] = -self.m*self.g*self.l_c*np.cos(x[0])/self.J
        Jac[1,1] = -(self.B/self.J + self.K_m**2/(self.J*self.R))
        return Jac
    
    # Nonlinear transition model from t_k to t_k+1...ie: dt
    def nonlin_transition_model(self, x,u=None):
        return ce.runge_kutta4_input(self.pend_ode, x, u, self.dt)
    
    # Kalman step
    def Kalman_step(self, z, u=None, taylor_order=2):
        """
        Take the variables and measurement for the Kalman filter

        Parameters
        -----
        z    : px1 vector
            measurement
        u    : mx1 vector
            control input
        
        Return
        -----
        nx1 nd.array
            propagated estimated states
        nxn nd.array
            propagated estimated variance
        """
        Jac_F = self.jacobian_pendulum_ode(self.x_kf)
        Phi_k, W_k = ce.discretize_nl_sys(Jac_F, self.Gamma_c, self.w_PSD, self.dt, taylor_order, with_Gamk = False, with_Wk = True)
        # Propagate covariance and state estimates
        self.P_kf = Phi_k @ self.P_kf @ Phi_k.T + W_k
        self.x_kf = self.nonlin_transition_model(self.x_kf, u)
        # Form Kalman Gain, update estimate and covariance
        K = self.P_kf @ self.H.T @ np.linalg.inv(self.H @ self.P_kf @ self.H.T + self.v_PSD)
        zbar = self.H @ self.x_kf
        # r = z - zbar      
        self.x_kf += K @ (z - zbar)      # K x r, r: residue
        self.P_kf = (self.idty - K @ self.H) @ self.P_kf @ (self.idty - K @ self.H).T + K @ self.v_PSD @ K.T
        self.xs_kf.append(self.x_kf.copy())
        self.Ps_kf.append(self.P_kf.copy())
        return self.x_kf, self.P_kf


    # Cauchy setting
    # This is the callback function correpsonding to the decription for point 1.) above 
    def dynamics_update(self, c_duc):
        taylor_order = 2
        pyduc = ce.Py_CauchyDynamicsUpdateContainer(c_duc)
        ## Propagate x 
        xk = pyduc.cget_x()
        u = pyduc.cget_u()
        xbar = self.nonlin_transition_model(xk) # propagate from k -> k+1
        pyduc.cset_x(xbar)
        pyduc.cset_is_xbar_set_for_ece() # need to call this!
        ## Phi, Gamma, beta may update
        Jac_F = self.jacobian_pendulum_ode(xk)
        Phi_k, Gam_k = ce.discretize_nl_sys(Jac_F, self.Gamma_c, None, self.dt, taylor_order, with_Gamk=True, with_Wk=False)
        pyduc.cset_Phi(Phi_k)
        pyduc.cset_Gamma(Gam_k)
        #pyduc.cset_beta(beta)

    # This is the callback function correpsonding to the decription for point 2.) above 
    def nonlinear_msmt_model(self, c_duc, c_zbar):
        pyduc = ce.Py_CauchyDynamicsUpdateContainer(c_duc)
        ## Set zbar
        xbar = pyduc.cget_x() # xbar
        zbar = self.H @ xbar # for other systems, call your nonlinear h(x) function
        pyduc.cset_zbar(c_zbar, zbar)

    # This is the callback function correpsonding to the decription for point 3.) above 
    def msmt_model_jacobian(self, c_duc):
        pyduc = ce.Py_CauchyDynamicsUpdateContainer(c_duc)
        ## Set H: for other systems, call your nonlinear jacobian function H(x)
        pyduc.cset_H(self.H) # we could write some if condition to only set this once, but its such a trivial overhead, who cares

    def cauchy_start(self, num_windows):
        self.num_windows = num_windows
        cauchyEst = ce.PySlidingWindowManager(self.system_type, num_windows, self.swm_print_debug,self.win_print_debug)
        self.cauchyEst = cauchyEst
        self.cauchyEst.initialize_nonlin(self.x0_cf, self.A0, self.p0, self.b0, self.beta, self.gamma, self.dynamics_update, self.nonlinear_msmt_model, self.msmt_model_jacobian, self.num_controls, self.dt)
        print(f"Start a Cauchy filter with a window size of {self.num_windows}. System type: {self.system_type}")

    def cauchy_step(self, z, u=None):
        return self.cauchyEst.step(z, u)
    
    def cauchy_shutdown(self):
        self.cauchyEst.shutdown()
    
    def process_noise_generation(self, size=1):
        """
        Generate process noise - Gaussian
        """
        if isinstance(size,int):
            return self.w_PSD**.5 * np.random.randn(size)
        return self.w_PSD**.5 * np.random.randn(size[0], size[1])
        

    def measurement_noise_generation(self, size=1):
        """
        Generate measurement noise - uniform ~ [-Enc_n, Enc_n]. \\
        This function is subject to change to other noise distributions.
        """
        return np.random.uniform(-self.Enc_n, self.Enc_n, size)

    def dynamic_simulation(self, propagations, noise=True):
        """
        Run the dynamic simulation with or without process/measurement noise
        Arguments:
        ----
        propagations:   int
            The length of the simulation
        noise:          boolean
            To use noises during propagations or not
        """
        xk = self.x0.reshape(-1)
        xs = [xk.copy()] # State vector history
        ws = []   # Process noise history
        # vs = [V[0]**0.5 * np.random.randn()] # Measurement noise history
        vs = [self.measurement_noise_generation()]
        zs = [self.H @ xk + vs[0]] # Measurement history
        # propagations = 160
        for k in range(propagations):
            if noise:
                wk = self.dt * self.process_noise_generation(1)
                vk = self.measurement_noise_generation(1)
            else:
                wk = np.array([0])
                vk = np.array([0])
            xk[1] += wk
            # xk = xk + wk * mp.Gamma_c
            xk = self.nonlin_transition_model(xk)
            xs.append(xk)
            ws.append(wk)
            zk = self.H @ xk + vk
            vs.append(vk)
            zs.append(zk)
        xs = np.array(xs)
        zs = np.array(zs)
        ws = np.array(ws)
        vs = np.array(vs)
        return xs, zs, ws, vs


    @staticmethod
    def saturation(x, lb, ub=None):
        if ub is None:
            lb = -abs(lb)
            ub =  abs(lb)
        if lb > ub:
            lb, ub = ub, lb
        if x > ub:
            x = ub
        if x < lb:
            x = lb
        return x
    


