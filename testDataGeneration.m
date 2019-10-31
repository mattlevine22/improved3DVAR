clc;
clear all;
close all;

H = [1 0];
noise_params = [0 1];
dt = 0.1;
v0 = [1.5 2]';
t0 = 0;
tf = 10;
[true_trajectory, observed_trajectory, time] = generateData(@PsiTest, H, noise_params, dt, t0, tf, v0);

plot3(time, true_trajectory(1,:), true_trajectory(2,:),'r')
hold on;
plot3(time, observed_trajectory(1,:), zeros(length(observed_trajectory(1,:))) , 'b')