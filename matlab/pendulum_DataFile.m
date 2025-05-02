% Define pendulum parameter for pendulum collision experiments and
% simulations

% pendulum configuration
global mp
mp = struct(...
    'g', 9.81, ... % gravitational constant
    'B', 8.1055e-6, ... % motor damping
    'L', 0.23e-3, ... % inductance
    'R', 3.85, ... % resistance
    'V_s', 10.7, ... % supply voltage
    'K_m', 0.0228, ... % motor constant
    'm', 0.03937, ... % mass of rod
    'l_c', 0.0254, ... % length of rod
    'J_motor', 1.67e-6, ...
    'J_rod', 2.12*1E-5, ...
    'sr', 1000, ... % sampling rate (Hz) % 100
    'w_PSD', 0., ... % process noise Power spectral density
    'EncRes', 400 ... % encoder resolution per revolution
);
% mp.J_rod = 1/12*mp.m; % *mp.l_c^2; % pendulum config; 1/12*m*l_c^2 for pendulum config
mp.J = mp.J_motor + mp.J_rod + mp.m * mp.l_c^2;
mp.dt = 1/mp.sr;
mp.Enc_n = 2 * pi / mp.EncRes; % encoder noise (uniform distributed noise)
mp.VelRes = mp.EncRes / mp.dt;