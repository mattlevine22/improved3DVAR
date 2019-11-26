%for data generation
function [true_trajectory, observed_trajectory, time, drivers] = generateData(Psi, H, noise_params, dt, t0, tf, v0, is_driven)
tic
% check if v0 is parameter or default
if(nargin <= 6) % default
    v0 = rand();
end

%unpack noise params
muObs = noise_params.obs_noise.mean;
Gamma = noise_params.obs_noise.covariance;
muState = noise_params.state_noise.mean;
Sigma = noise_params.state_noise.covariance;

%get dimensions of data and observations
d = size(H,2);
n = size(H,1);

%init time and trajectories
time = t0:dt:tf;
true_trajectory = zeros(d, length(time));
observed_trajectory = zeros(n, length(time));

%simulate nutrition driver
drivers = zeros(1,2);
if is_driven
    t_now = 0;
    tUB = 10*60;
    tLB = 6*60;
    carbUB = 100000;
    carbLB = 5000;
    d_counter = 0;
    while t_now < tf
        d_counter = d_counter + 1;
        t_now = t_now + (tLB + (tUB - tLB)*rand);
        carb_now = carbLB + (carbUB - carbLB)*rand;
        drivers(d_counter,:) = [t_now, carb_now];
    end
end

%fill first value of trajectories
t_init = t0;
true_trajectory(:,1) = Psi(v0, t_init, dt, drivers) + mvnrnd(muState, Sigma);
observed_trajectory(:,1) = H*true_trajectory(:,1) + mvnrnd(muObs, Gamma);

%fill whole trajectories
for i=2:length(time)
    t_init = t_init + dt;
    true_trajectory(:,i) = Psi(true_trajectory(:,i-1), t_init, dt, drivers) + mvnrnd(muState, Sigma);
    observed_trajectory(:,i) = (H*true_trajectory(:,i)) + mvnrnd(muObs, Gamma);
end
disp('for data generation: ')
toc
end