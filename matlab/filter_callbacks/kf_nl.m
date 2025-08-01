function [x_kf_new,P_kf_new] = kf_nl(mp, x_kf, P_kf, z)
Gamma_c = mp.Gamma_c;
W = mp.w_PSD;
V = mp.v_PSD;
dt = mp.dt;
H = mp.H;
I = eye(2);

disc_taylor_order = 2;
% Jac_F = jacobian_mp_ode(x_kf);

Jac = zeros(2);
Jac(1,2) = 1;
Jac(2,1) = -mp.m*mp.g*mp.l_c*cos(x_kf(1))/mp.J;
Jac(2,2) = -(mp.B/mp.J + mp.K_m^2/(mp.J*mp.R));

Jac_F = Jac;

[Phi_k, W_k] = discretize_nl_sys(Jac_F, Gamma_c, W, dt, disc_taylor_order, false, true);
% Propagate covariance and state estimates
P_kf = Phi_k * P_kf * Phi_k' + W_k;
% x_kf = nonlin_transition_model(x_kf);
x_kf = runge_kutta4(@(x_kf)motor_pend_ode(mp,x_kf), x_kf, dt);

% Form Kalman Gain, update estimate and covariance
K = (H * P_kf * H' + V) \ (H * P_kf)';
zbar = H * x_kf;            % estimated measurement
r = z - zbar;         % residue
x_kf = x_kf + K * r;        % updated state
P_kf = (I - K * H) * P_kf * (I - K * H)' + K * V * K';    % covariance update

% Store estimates
x_kf_new = x_kf;
P_kf_new = P_kf;

end

function dx_dt = motor_pend_ode(mp,x)
    dx_dt = zeros(2, 1);
    % x1 = theta, x2 = omega
    dx_dt(1) = x(2);
    dx_dt(2) = -(mp.B/mp.J + mp.K_m^2/(mp.J*mp.R))*x(2)-mp.m*mp.g*mp.l_c*sin(x(1))/mp.J; % currently u does not depend on x
end
