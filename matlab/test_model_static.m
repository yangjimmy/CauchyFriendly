epsilon = 1e-6;

A_k1 = [epsilon 0; 0 epsilon];
A_k2 = -[epsilon 0; 0 epsilon];

x = [0;0];

xs = [x];

for i = 1:1000
    if mod(i,2) == 0
        x = A_k1*x+epsilon*randn(2,1);
    else
        x = A_k2*x+epsilon*randn(2,1);
    end
    xs = [xs x];
end

plot(1:1000, xs(1,1:1000))