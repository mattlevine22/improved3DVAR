% 1 file per Psi

e.g.

function v1 = L63(v0, dt)
end

% 1 file for data generation

function [true_trajectory, observed_trajectory] = data(Psi, H, noise_params, dt, v0=DEFAULTS)
end

% 1 file for running 3DVAR

function state_assim_now = ThreeDvar_step(state_pred_now, meas_now, K)
	state_assim_now = 0;
end


function foo = fullstateLoss(state_pred_now, meas_now, K, true_state)
	foo = norm(ThreeDvar_step(state_pred_now, meas_now, K) - true_state);
end

% 1 file per K-learning-method
function x = GDfullstates(m0, true_traj, obs_traj, dt, K0, learning_rate, Psi, loss)

	N = len(true_traj);
	d = len(m0);
	m_assim = zeros(N,d);
	m_pred = zeros(N,d);
	K_list = zeros(N,shape(K0));
	L = zeros(N,1);

	state_pred_now = psi(m0, dt);
	K = K0; % needs a comment
	for j=1:N
		meas_now = obs_traj[j,:];
		% need to compute this partial derivative!!!
		K = K - learning_rate*partial_derivative(loss(state_pred_now, meas_now, K, true_traj[j,:]),'K');
		K_list[j,:] = K;

		% now, use new K in 3dvar
		m_assim[j,:] = ThreeDvar_step(state_pred_now, meas_now, K);
		L[j] = loss(state_pred_now, meas_now, K, true_state); % check that this is L2 norm

		state_pred_now = psi(m_assim[j,:], dt);
	end

end
