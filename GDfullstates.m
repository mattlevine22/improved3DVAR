function Kopt = GDfullstates(m0, true_traj, obs_traj, dt, K0, learning_rate, Psi, H)

N = length(true_traj);
d = length(m0);
m_assim = zeros(N,d);
m_pred = zeros(N,d);
K_list = zeros(N,size(K0));
L = zeros(N,1);

state_pred_now = psi(m0, dt);
K = K0;
for j=1:N
    m_pred(j,:) = state_pred_now;
    
    meas_now = obs_traj(j,:);
    % need to compute this partial derivative!!!
    a = meas_now - H*state_pred_now;
    b = state_pred_now - true_traj(j,:);
    dPsi = 2*(K*a + b)*a';
    
    K = K - learning_rate*dPsi;
    K_list(j,:) = K;
    
    % now, use new K in 3dvar
    m_assim(j,:) = ThreeDvar_step(state_pred_now, meas_now, K, H);
    L(j) = norm(m_assim(j,:) - true_state);
    
    state_pred_now = Psi(m_assim(j,:), dt);
end

Kopt = K;

end

