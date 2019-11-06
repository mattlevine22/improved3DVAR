clc;
clear all;
close all;

%init parameter and generate trajectories
H = [1 0 0];
dt = 0.001;
noise_params = struct();
noise_params.mean = 0;
noise_params.variance = 1;
v0 = get_lorenz_inits();
t0 = 0;
tf = 25;
[true_trajectory, observed_trajectory, time] = generateData(@PsiL63, H, noise_params, dt, t0, tf, v0);

%example use of EnKF
noise_params_EnKF = noise_params;
number_Particles = 50; %how many?
v0 = get_lorenz_inits();
est_traj = EnKF(v0, observed_trajectory, dt, noise_params_EnKF, number_Particles, @PsiL63, H);
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
K0 = 0.1*randn(sizes(2),sizes(1));%[0.08, 0.12 ,0.003]';
%for OptObservations the starting point is crucial, better start with known to be good value
learning_rate = 0.0005;
m0 = get_lorenz_inits();
%gradient descent learning
Kopt = GDfullstates(m0, true_trajectory, observed_trajectory, dt, K0, learning_rate, @PsiL63, H);
%learning on whole trajectory
%Kopt = OptObservations(m0, observed_trajectory, dt, K0, @PsiL63, H);

%validate Kopt on N_tests unseen trajectories
N_tests = 2;
validate(Kopt, N_tests, @PsiL63, dt, H, noise_params, @get_lorenz_inits)