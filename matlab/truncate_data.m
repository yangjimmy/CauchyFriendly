load("data/pendulum_wall_0603_01.mat")

t = simData.pos.Time;
start_idx = find(t==8.965);

t = t(start_idx:end)-t(start_idx);
vel = simData.vel.Data(start_idx:end);
pos = simData.pos.Data(start_idx:end);
% scale the position and velocity (incorrect readings + offset)
vel = (vel-min(abs(vel)))/(max(pos)-min(abs(pos)));
pos = (pos-min(abs(pos)))/(max(pos)-min(abs(pos))) * pi/2;

save("data\pendulum_wall_0603_01_truncated.mat","t","pos","vel")