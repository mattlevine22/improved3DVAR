% 1 function for running full 3DVAR
function [m_assim, m_pred] = Full3DVAR(m0, K, Psi, obs_traj, H, dt)
	N = len(obs_traj);
	d = len(m0);
	m_assim = zeros(N,d);
	m_pred = zeros(N,d);

	m_pred(1,:) = Psi(m0, dt);
    
	for j=1:N
		m_assim(j,:) = ThreeDvar_step(m_pred(j,:), obs_traj(j,:), K, H);
		if j<N
			m_pred(j+1,:) = psi(m_assim(j,:), dt);
		end
	end
end

% 1 function for running 3DVAR single step
function state_assim_now = ThreeDvar_step(state_pred_now, meas_now, K, H)
    N = len(H);
    %state_pred_now is Psi(mk)
	state_assim_now = (eye(N)-K*H)*state_pred_now - K * meas_now;
end