%sigma has to be positive, (C,A) has to be detectable
epsilon = 1;
Qn = 0; %E(WW')
Rn = epsilon * eye(1); %E(VV') (sigma)
Nn = 0; %E(WV') (indep here)
%A = [1 0 0; 0 1 1; 1 2 3];
%B = [0; 0; 0]; %(no input)
%C = [1 0 0]; %H
%D = 0; %(no input)
A = [1.1269   -0.4940    0.1129;
     1.0000         0         0;
          0    1.0000         0];
B = [-0.3832;
      0.5919;
      0.5191];
C = [1 0 0];
D = 0;
G=[0; 0; 0];
H=0;
SYS=ss(A,[B G],C,[D H]);
[kest,L,P] = kalman(SYS,Qn,Rn,Nn);
Kric = (L\A)'

%init parameter and generate trajectories
H = C;%[1 0 0];
is_driven = 0;
dt = 0.01;
noise_params = struct();
noise_params.state_noise.mean = 0;
noise_params.state_noise.covariance = 0;
noise_params.obs_noise.mean = 0;
noise_params.obs_noise.covariance = 1;

v0 = get_lorenz_inits();
t0 = 0;
tf = 10;
[true_trajectory, observed_trajectory, time, drivers] = generateData(@PsiLinear, H, noise_params, dt, t0, tf, v0, is_driven);

%example use of EnKF
noise_params_EnKF = noise_params;
noise_params_EnKF.state_noise.mean = zeros(length(H),1);
noise_params_EnKF.state_noise.covariance = eye(length(H));
number_Particles = 50; %how many?
v0 = get_lorenz_inits();
[est_traj, Kenkf] = EnKF(v0, observed_trajectory, dt, noise_params_EnKF, number_Particles, @PsiLinear, H, drivers);
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
Kgd = GDfullstates(m0, true_trajectory, observed_trajectory, dt, K0, learning_rate, @PsiLinear, H, drivers);
%Kopt = [0.08, 0.12 ,0.003]';
%Ktr = OptObservations(m0, observed_trajectory, dt, K0, @PsiLinear, H, drivers);

Kric
Kenkf
Kgd
Ktr

%validate Kopt on N_tests unseen trajectories
N_tests = 1;
t0_test = t0;
tf_test = tf;
validate_new(1,Kric, N_tests, @PsiLinear, dt, t0_test, tf, tf_test, H, noise_params, @get_lorenz_inits, is_driven)
validate_new(1,Kenkf, N_tests, @PsiLinear, dt, t0_test, tf, tf_test, H, noise_params, @get_lorenz_inits, is_driven)
validate_new(1,Kgd, N_tests, @PsiLinear, dt, t0_test, tf, tf_test, H, noise_params, @get_lorenz_inits, is_driven)
%validate_new(1,Ktr, N_tests, @PsiLinear, dt, t0_test, tf, tf_test, H, noise_params, @get_lorenz_inits, is_driven)
