clc;
clear all;
close all;

H = [1 0 0];
noise_params = struct();
noise_params.mean = 0;
noise_params.variance = 1;
dt = 0.01;
v0 = get_lorenz_inits();
t0 = 0;
tf = 50;
[true_trajectory, observed_trajectory, time] = generateData(@PsiL63, H, noise_params, dt, t0, tf, v0);

figure(1)
plot3(true_trajectory(1,:), true_trajectory(2,:),true_trajectory(3,:),'r')

figure(2)
plot(time, observed_trajectory, 'b')

sizes = size(H);
K0 = 0.1*randn(sizes(2),sizes(1));
learning_rate = 0.0005;
m0 = get_lorenz_inits();
Kopt = GDfullstates(m0, true_trajectory, observed_trajectory, dt, K0, learning_rate, @PsiL63, H);

N_tests = 1;
validate(Kopt, N_tests, @PsiL63, dt, H, noise_params, @get_lorenz_inits)