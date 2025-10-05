% Define pendulum parameter for pendulum collision experiments and
% simulations

% pendulum configuration
global mp
mp = struct(...
    'g', 9.81, ... % gravitational constant
    'B', 8.1055e-6, ... % motor damping
    'L', .23e-3, ... % inductance .23e-3
    'R', 3.85, ... % resistance
    'V_s', 10.7, ... % supply voltage
    'K_m', 0.0228, ... % motor constant
    'm', 0.03937, ... % mass of rod 0.03937
    'l_c', 0.0254, ... % length of rod 0.0254
    'J_motor', 1.67e-6, ...
    'J_rod', 2.12*1E-5, ...
    'sr', 250, ... % sampling rate (Hz) % 100'
    'w_PSD', 0.01, ... % process noise Power spectral density
    'EncRes', 400 ... % encoder resolution per revolution
);
% mp.J_rod = 1/12*mp.m; % *mp.l_c^2; % pendulum config; 1/12*m*l_c^2 for pendulum config
mp.J = mp.J_motor + mp.J_rod + mp.m * mp.l_c^2;
mp.dt = 1/mp.sr;
mp.Enc_n = 2 * pi / mp.EncRes; % encoder noise (uniform distributed noise)
mp.VelRes = mp.EncRes / mp.dt;

% model
mp.H = [1.0, 0.0];             % meausrement model
mp.Gamma_c = [0.0; 1.0];       % Continuous time Gamma (\Gamma_c), multiplies process noise w

% ---- Measurement noise setting
% uniform noise [-a,a] -> beta: a/beta = 2.33
% a / beta ratio = 2.3305
% mp.scale_u2c = 1.0/2.3305;
mp.scale_u2c = 1/sqrt(3);
mp.gamma = mp.Enc_n * mp.scale_u2c;

% uniform noise [-a,a] -> sigma: a/sigma = 1.7319
% a / sigma ratio = 1.7319
mp.scale_u2g = 1/(sqrt(-sqrt(2)*log(sqrt(2)/4)));
% mp.scale_u2g = 1.0/1.7319;
mp.v_PSD = (mp.Enc_n*mp.scale_u2g)^2;

% ---- Process noise setting
mp.scale_g2c = 1.0/1.3898;
mp.scale_c2g = 1.3898;

% mp.beta = 4;
% mp.w_PSD = (mp.beta * mp.scale_c2g)^2 * mp.dt;
% mp.w_PSD = 3.8^2;
% mp.w_PSD = 120^2;
mp.w_PSD = 15^2;
mp.beta = sqrt(mp.w_PSD/mp.dt) * mp.scale_g2c;

% ----- Archive
% PSD setting
% mp.v_PSD = (2*mp.Enc_n)^2 /12 / mp.dt;
% mp.v_PSD = mp.Enc_n^2 /12 / mp.dt;
% mp.w_PSD = mp.w_PSD / mp.dt;