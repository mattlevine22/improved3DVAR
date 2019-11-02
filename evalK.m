function error = evalK(K, m0, obs_traj, dt,Psi, H)
N = size(obs_traj,2);
d = length(m0);
%m_hat = zeros(N,d);
error = 0;%(obs_traj(:,1)-H*Psi(m0,dt)).^2; %should we penalize first point?
[m_assim, m_pred] = Full3DVAR(m0, K, Psi, obs_traj, H, dt);
for i=1:N-1
   error = error + (obs_traj(:,i+1)-H*Psi(m_assim(i,:),dt)).^2;
   %m_hat(i+1,:) = ThreeDvar_step(Psi(m_hat(i,:),dt), obs_traj(:,i+1), K, H);

end
end
