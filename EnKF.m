function est_traj = EnKF(v0, obs_traj, dt, noise_params, number_Particles, Psi, H) %v0 one start value
%unpack noise params
mu = noise_params.mean;
Gamma = noise_params.variance;

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

%get first set of particles from given start state
v(1,:,:) = v0 + random('Normal',mu,Gamma, 1, d, number_Particles);


for j=1:N
    %prediction
    for p = 1:number_Particles %particle wise:
        v_hat(j,:,p) = Psi(v(j,:,p),dt) + random('Normal',mu,Gamma, d, 1);
    end
    
    m_hat(j,:) = 1/number_Particles * sum(v_hat(j,:,:),3); %check sum
    
    for p = 1:number_Particles  %particle wise:
        helper = (v_hat(j,:,p)-m_hat(p,:));
        C_hat = C_hat + sum(helper*helper'); %outer product, check sum
    end
    C_hat = C_hat/number_Particles;
    
    %analysis
    S = H * C_hat * H' + Gamma; %index j+1
    K = C_hat * H' * inv(S); %index j+1
    y(j,:,:) = obs_traj(:,j) + random('Normal',mu,Gamma,1, n, number_Particles);%s??
    for p = 1:number_Particles
        v(j,:,p) = (eye(d) - K*H)*v_hat(j,:,p)' + K * y(j,:,p);
    end
end


%generate estimated trajectory (expected value of final particles)
est_traj = sum(v,3)'/number_Particles;

end