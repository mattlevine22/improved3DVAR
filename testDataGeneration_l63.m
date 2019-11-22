clc;
clear all;
close all;

useEnKF = 0;
manyPlots = 0;

%init parameter and generate trajectories
H = [1 0 0];
is_driven = 0;
dt = 0.01;
t0 = 0;
tf = 1;
noise_params = struct();
noise_params.state_noise.mean = 0;
noise_params.state_noise.covariance = 0;
noise_params.obs_noise.mean = 0;
noise_params.obs_noise.covariance = 1;

runs = 1;
gainValues = zeros(runs,1);
gainValues2 = zeros(runs,1);

for j = 1:runs
j
v0 = get_lorenz_inits();
[true_trajectory, observed_trajectory, time, drivers] = generateData(@PsiL63, H, noise_params, dt, t0, tf, v0, is_driven);

%example use of EnKF
if(useEnKF)
    noise_params_EnKF = noise_params;
    noise_params_EnKF.state_noise.mean = zeros(length(H),1);
    noise_params_EnKF.state_noise.covariance = eye(length(H));
    number_Particles = 50; %how many?
    v0 = get_lorenz_inits();
    [est_traj,Kenkf] = EnKF(v0, observed_trajectory, dt, noise_params_EnKF, number_Particles, @PsiL63, H, drivers);

    %visualization of EnKF
    N=length(observed_trajectory);
    figure(8)
    for i=1:3
        subplot(3,1,i)
        plot(dt*(1:N),true_trajectory(i,:),'-b'); hold on;
        plot(dt*(1:N),est_traj(i,:),'--');
        legend('true', 'est')
    end
    subplot(3,1,1)
    plot(dt*(1:N),observed_trajectory(:),'-g')
    true_trajectory = est_traj;
end

if(manyPlots)
    fig = figure;
    plot3(true_trajectory(1,:), true_trajectory(2,:),true_trajectory(3,:),'r')
    title('Full trajectory (ground truth) of L63')
    legend('full states')
    xlabel('x')
    ylabel('y')
    zlabel('z')
    saveas(fig,[pwd '/plots/fullTrajectoryL63.png'])
    savefig(fig, [pwd '/plots/fullTrajectoryL63.fig'])

    fig = figure;
    plot(time, observed_trajectory, 'b')
    title('Observed trajectory (partial, noisy) of L63')
    legend('observed y')
    xlabel('t')
    ylabel('y')
    saveas(fig,[pwd '/plots/observedTrajectoryL63.png'])
    savefig(fig,[pwd '/plots/observedTrajectoryL63.fig'])
   
end

sizes = size(H);
% K0 = [0.08, 0.12 ,0.003]';
K0 = 0.1*randn(sizes(2),sizes(1));
%for OptObservations the starting point is crucial, better start with known to be good value
learning_rate = 0.0005;
m0 = get_lorenz_inits();
KoptGD = GDfullstates(m0, true_trajectory, observed_trajectory, dt, K0, learning_rate, @PsiL63, H, drivers);
%Kopt = [0.08, 0.12 ,0.003]';
errorKoptGD = evalK(KoptGD, m0, observed_trajectory, dt,@PsiL63, H, drivers)

tic
%KoptOpt = OptObservations(m0, observed_trajectory, dt, KoptGD, @PsiL63, H, drivers);
disp('For the optimization: ')
toc

%errorKoptOpt = evalK(KoptOpt, m0, observed_trajectory, dt,@PsiL63, H, drivers)

%gainValues(j) = Kopt(1);
%gainValues2(j) = Kopt(2);
end
%figure();
%plot(1:runs,gainValues, 'xb')
%fig=figure;
%histogram(gainValues)
%hold on;
%m = mean(gainValues);
%plot([m m],[0 runs/2],'-r')
%savefig(fig, 'figname.fig');

%validate Kopt on N_tests unseen trajectories
N_tests = 1;
t0_test = t0;
tf_test = 10;
tf_future = 10;
output = validate_new(1,KoptGD, N_tests, @PsiL63, dt, t0_test, tf_test, tf_future, H, noise_params, @get_lorenz_inits, is_driven)
%validate_new(1,KoptOpt, N_tests, @PsiL63, dt, t0_test, tf_test, tf_future, H, noise_params, @get_lorenz_inits, is_driven)