B = 0; % motor damping
L = 0.23e-3; % inductance
R = 3.85; % resistance
V_s = 10.7; % supply voltage
K_m = 0.24; % motor constant
m = 0.044; % mass of rod
l_c = 0.077; % length of rod
J_motor = 1.67e-6 ;
J_rod = 1/12*m*l_c^2; % rotor config; 1/3*m*l_c^2 for pendulum config
J = J_motor + J_rod;

A_c = [0 1; -(B/J + K_m^2/(J*R)) 0];

eig(A_c)
tf = 2;
integral(@(t) expm(-A_c*t), 0,tf,'ArrayValued', true)
