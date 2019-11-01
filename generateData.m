%for data generation
function [true_trajectory, observed_trajectory, time] = generateData(Psi, H, noise_params, dt, t0, tf, v0)

% check if v0 is parameter or default
if(nargin <= 6) % default
    v0 = rand();
end

%unpack noise params
mu = noise_params.mean;
sigma = noise_params.sigma;

%get dimensions of data and observations
d = size(H,2);
n = size(H,1);

%init time and trajectories 
time = t0:dt:tf;
true_trajectory = zeros(d, length(time));
observed_trajectory = zeros(n, length(time));

%fill first value of trajectories
true_trajectory(:,1) = Psi(v0, dt);
observed_trajectory(:,1) = H*true_trajectory(:,1) + random('Normal',mu,sigma, n, 1);

%fill whole trajectories
for i=2:length(time)
    true_trajectory(:,i) = Psi(true_trajectory(:,i-1), dt);
    observed_trajectory(:,i) = (H*true_trajectory(:,i)) + random('Normal',mu,sigma, n, 1);
end

end