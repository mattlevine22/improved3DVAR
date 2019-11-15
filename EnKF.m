function [est_traj,Kavg] = EnKF(v0, obs_traj, dt, noise_params, number_Particles, Psi, H, drivers) %v0 one start value
tic
%unpack noise params
muObs = noise_params.obs_noise.mean;
Gamma = noise_params.obs_noise.covariance;
muState = noise_params.state_noise.mean;
Sigma = noise_params.state_noise.covariance;

%initialization
n = size(H,1); %dim of observations
d =  size(H,2); %dim of state
N = size(obs_traj,2);
v = zeros(N,d, number_Particles);
v_hat = zeros(N,d, number_Particles);
m_hat = zeros(N,d);
C_hat = zeros(d,d); %not going to save all Cs?
S = zeros(n,n);
K = zeros(d,n);
y = zeros(N,n, number_Particles);
est_traj = zeros(d,N);
C = zeros(N,1);
K_list = zeros(N,d,n);
Kavg = 0;

%get first set of particles from given start state
v(1,:,:) = v0 + mvnrnd(muState, Sigma, number_Particles)';


for j=1:N-1
    %prediction
    for p = 1:number_Particles %particle wise:
        v_hat(j+1,:,p) = Psi(v(j,:,p),dt,drivers) + mvnrnd(muState,Sigma)';
    end

    m_hat(j+1,:) = 1/number_Particles * sum(v_hat(j+1,:,:),3); %check sum
    C_hat = zeros(d,d);
    for p = 1:number_Particles  %particle wise:
        helper = -(v_hat(j+1,:,p)-m_hat(j+1,:))';
        C_hat = C_hat + helper*helper'; %outer product, check sum
    end
    C_hat = C_hat/number_Particles;
    C(j) = norm(C_hat);
    %analysis
    S = H * C_hat * H' + Gamma; %index j+1
    K = C_hat * H' / S; %index j+1
    if(j > 0.8*(N-1))
       Kavg = Kavg + K; 
    end
    %K_list(j,:) = K; %isnt that a learned K as well?
%     y(j+1,:,:) = obs_traj(:,j+1)' + random('Normal',mu,Gamma,1, n, number_Particles);%s??
    y(j+1,:,:) = obs_traj(:,j+1)' + mvnrnd(muObs, Gamma, number_Particles);%is this right??
    for p = 1:number_Particles
        v(j+1,:,p) = ((eye(d) - K*H)*v_hat(j+1,:,p)' + K * y(j+1,:,p)')';
    end
end

%average last 20% of Ks
Kavg = Kavg/(floor(0.2*(N-1)));
%generate estimated trajectory (expected value of final particles)
est_traj = sum(v,3)'/number_Particles;

disp('For EnKF: ')
toc
end