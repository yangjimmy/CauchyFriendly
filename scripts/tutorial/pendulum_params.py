#!/usr/bin/env python
# coding: utf-8

# Pendulum dynamics equation
# \begin{align}
# \dfrac{dx_1}{dt} &= x_2 \\
# \dfrac{dx_2}{dt} &= -\frac{m g l_c}{J} sin(x_1) - (\frac{B}{J} + \frac{K_m^2}{JR}) x_2
# \end{align}

import numpy as np

# ----- Define system parameters -----
class PendulumParams:    
    def __init__(self, dt=0.001):
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
        self.w_PSD = .01
        self.EncRes = 400
        self.w_PSD = .01
        self.H = np.array([1.0,0.0])
        self.Gamma_c = np.array([[0.0],[1.0]])
        
        self.dt = dt
        self.sr = 1/self.dt
        
        self.J = self.J_motor + self.J_rod
        self.Enc_n = 2 * np.pi / self.EncRes
        self.VelRes = self.EncRes / self.dt
        self.v_PSD = self.Enc_n**2 / 12 / self.dt
        self.w_PSD = self.w_PSD / self.dt
