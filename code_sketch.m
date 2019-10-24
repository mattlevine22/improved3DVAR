% 1 file per Psi

e.g.

function v1 = L63(v0, dt)
end

% 1 file for data generation

function [true_trajectory, observed_trajectory] = data(Psi, H, noise_params, dt, v0=DEFAULTS)
end

% 1 function for running 3DVAR single step
function state_assim_now = ThreeDvar_step(state_pred_now, meas_now, K, H)
	state_assim_now = 0;
end

% 1 function for running full 3DVAR
function [m_assim, m_pred] = Full3DVAR(m0, K, Psi, obs_traj, H, dt)
	N = len(obs_traj);
	d = len(m0);
	m_assim = zeros(N,d);
	m_pred = zeros(N,d);

	m_pred[1,:] = Psi(m0, dt);
	for j=1:N
		m_assim[j,:] = ThreeDvar_step(m_pred[j,:], obs_traj[j,:], K, H)
		if j<N
			m_pred[j+1,:] = psi(m_assim[j,:], dt);
		end
	end
end

% loss function
function foo = fullstateLoss(state_pred_now, meas_now, K, H, true_state)
	foo = norm(ThreeDvar_step(state_pred_now, meas_now, K, H) - true_state);
end

% 1 file per K-learning-method
function x = GDfullstates(m0, true_traj, obs_traj, dt, K0, learning_rate, Psi, H, loss)

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
		m_assim[j,:] = ThreeDvar_step(state_pred_now, meas_now, K, H);
		L[j] = loss(state_pred_now, meas_now, K, true_state); % check that this is L2 norm

		state_pred_now = psi(m_assim[j,:], dt);
	end
end


% for each Psi, we want a sample_inits functions

function get_lorenz_inits()
	init = rand(lorenz_box);
end

% validation code

function validate(Kopt, N_tests, Psi, dt, H, noise_params, sample_inits)

	example_metric = zeros(N_tests,1);
	for k=1:N_tests
		% get v0 IC for Psi
		v0 = sample_inits();
		% generate data from Psi
		[true_trajectory, observed_trajectory] = data(Psi, H, noise_params, dt, v0);

		% get m0 IC for 3DVAR
		m0 = sample_inits();
		[m_assim, m_pred] = Full3DVAR(m0, Kopt, Psi, observed_trajectory, H, dt)

		% compare assim vs TRUE
		example_metric[k] = mean((m_assim - true_trajectory).^2)

	% Make Plots
end



