clc;
clear all;
close all;

%init parameter and generate trajectories
H = [1 0 0];
is_driven = 0;
dt = 0.01;
noise_params = struct();
noise_params.state_noise.mean = 0;
noise_params.state_noise.covariance = 0;
noise_params.obs_noise.mean = 0;
noise_params.obs_noise.covariance = 1;

v0 = get_lorenz_inits();
t0 = 0;
tf = 5;
[true_trajectory, observed_trajectory, time, drivers] = generateData(@PsiL63, H, noise_params, dt, t0, tf, v0, is_driven);

%example use of EnKF
noise_params_EnKF = noise_params;
noise_params_EnKF.state_noise.mean = zeros(length(H),1);
noise_params_EnKF.state_noise.covariance = eye(length(H));
number_Particles = 50; %how many?
v0 = get_lorenz_inits();
est_traj = EnKF(v0, observed_trajectory, dt, noise_params_EnKF, number_Particles, @PsiL63, H, drivers);
%true_trajectory = est_traj;

%visualization of EnKF
N=length(observed_trajectory);
figure(8)
for i=1:3
    subplot(3,1,i)
    plot(dt*(1:N),true_trajectory(i,:),'-b'); hold on;
    plot(dt*(1:N),est_traj(i,:),'--');
    legend('true', 'est')
end

%figure(3);
%plot3(true_trajectory(1,:), true_trajectory(2,:),true_trajectory(3,:),'r')

%figure(4);
%plot(time, observed_trajectory, 'b')

sizes = size(H);
% K0 = [0.08, 0.12 ,0.003]';
K0 = 0.1*randn(sizes(2),sizes(1));
%for OptObservations the starting point is crucial, better start with known to be good value
learning_rate = 0.0005;
m0 = get_lorenz_inits();
Kopt = GDfullstates(m0, true_trajectory, observed_trajectory, dt, K0, learning_rate, @PsiL63, H, drivers);
% Kopt = OptObservations(m0, observed_trajectory, dt, K0, @PsiL63, H, drivers);

%validate Kopt on N_tests unseen trajectories
N_tests = 2;
t0_test = t0;
tf_test = tf;
validate(Kopt, N_tests, @PsiL63, dt, t0_test, tf_test, H, noise_params, @get_lorenz_inits, is_driven)