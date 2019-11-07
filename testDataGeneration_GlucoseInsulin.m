clc;
clear all;
close all;

H = [0 0 1 0 0 0];
is_driven = 1;
noise_params = struct();
noise_params.state_noise.mean = 0;
noise_params.state_noise.covariance = 0;
noise_params.obs_noise.mean = 0;
noise_params.obs_noise.covariance = 100;


dt = 25; % sample every 5 minutes
v0 = get_glucose_insulin_inits();
t0 = 0;
tf = 7*24*60; % 1 week, in minutes
[true_trajectory, observed_trajectory, time, drivers] = generateData(@PsiGlucoseInsulin, H, noise_params, dt, t0, tf, v0, is_driven);

figure(3);
plot3(true_trajectory(1,:), true_trajectory(2,:),true_trajectory(3,:),'r')

figure(4);
plot(time, observed_trajectory, 'b')

sizes = size(H);
% K0 = [0.08, 0.12 ,0.003]';
K0 = 0.1*randn(sizes(2),sizes(1));
%for OptObservations the starting point is crucial, better start with known to be good value
learning_rate = 0.0005;
m0 = get_glucose_insulin_inits();
Kopt = GDfullstates(m0, true_trajectory, observed_trajectory, dt, K0, learning_rate, @PsiGlucoseInsulin, H, drivers);
% Kopt = OptObservations(m0, observed_trajectory, dt, K0, @PsiGlucoseInsulin, H, drivers);

N_tests = 2;
t0_test = t0;
tf_test = tf;
validate(Kopt, N_tests, @PsiGlucoseInsulin, dt, t0_test, tf_test, H, noise_params, @get_glucose_insulin_inits, is_driven)