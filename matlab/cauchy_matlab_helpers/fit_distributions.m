% addpath("cauchy_matlab_helpers\")

a = pi/200;

samples = random("Uniform",-a,a,100000,1);

% fit a Cauchy
[cauchy_params,~] = cauchyfit(samples,'info2');

% fit a Gaussian
[mu,sigma] = normfit(samples);

%%
x= -3:0.0001:3;
plot(x, cauchypdf(x,cauchy_params(1),cauchy_params(2))); hold on;
plot(x,normpdf(x,mu,sigma));
plot(x,unifpdf(x,-a,a)); hold off;
legend("Cauchy fit","Gaussian fit","Uniform")
grid on;
xlim([-0.05, 0.05]);


sprintf("Cauchy Distribution: a: %f, b: %f", cauchy_params(1), cauchy_params(2))

sprintf("Normal Distribution: mu: %f, sigma: %f", mu, sigma)