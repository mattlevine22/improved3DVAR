clc;
clear all;
close all;

H = [1 0 0];
noise_params = [0 1 0];
dt = 0.01;
v0 = [10, 20, 30]';
t0 = 0;
tf = 10;
[true_trajectory, observed_trajectory, time] = generateData(@PsiL63, H, noise_params, dt, t0, tf, v0);

plot3(time, true_trajectory(1,:), true_trajectory(2,:),'r')
hold on;
plot3(time, observed_trajectory(1,:), zeros(length(observed_trajectory(1,:))) , 'b')