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

figure(1)
plot3(true_trajectory(1,:), true_trajectory(2,:),true_trajectory(3,:),'r')

figure(2)
plot(time, observed_trajectory, 'b')