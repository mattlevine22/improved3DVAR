function Kopt = GDfullstates(m0, true_traj, obs_traj, dt, K0, learning_rate, Psi, H, drivers, times)
tic
N = size(true_traj,2);
d = length(m0);
m_assim = zeros(N,d);
m_pred = zeros(N,d);
K_list = zeros(N,size(K0,1),size(K0,2));
L = zeros(N,1);

if nargin==9
    times = zeros(1,N);
end

state_pred_now = Psi(m0, times(1), dt, drivers);
K = K0;
for j=1:N
    m_pred(j,:) = state_pred_now;

    meas_now = obs_traj(:,j);
    % need to compute this partial derivative!!!
    a = meas_now - H*state_pred_now;
    b = state_pred_now - true_traj(:,j);
    dPsi = 2*(K*a + b)*a';

    K = K - learning_rate*dPsi;
    K_list(j,:) = K;

    % now, use new K in 3dvar
    m_assim(j,:) = ThreeDvar_step(state_pred_now, meas_now, K, H);
    L(j) = norm(m_assim(j,:) - true_traj(:,j));

    state_pred_now = Psi(m_assim(j,:)', times(j), dt, drivers);
end

Kopt = K;

if(0)
figure;
subplot(2,1,1)
plot(L);
subplot(2,1,2)
leg_names = {};
for i=1:length(K)
    plot(dt*(1:N),K_list(:,i));hold on;
    leg_names{i} = sprintf('K_%.0f',i);
end
legend(leg_names)
end
disp('For Gradient Descent: ')
toc
end

