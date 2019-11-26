% 1 function for running full 3DVAR
function [m_assim, m_pred] = Full3DVAR(m0, K, Psi, obs_traj, H, dt, drivers, times)
	N = length(obs_traj);
	d = length(m0);
	m_assim = zeros(N,d);
	m_pred = zeros(N,d);

    if nargin==7
        times = zeros(1,N);
    end
    
    m_pred(1,:) = Psi(m0, times(1), dt, drivers);
    
	for j=1:N
		m_assim(j,:) = ThreeDvar_step(m_pred(j,:)', obs_traj(:,j), K, H);
		if j<N
			m_pred(j+1,:) = Psi(m_assim(j,:)', times(j), dt, drivers);
		end
	end
end