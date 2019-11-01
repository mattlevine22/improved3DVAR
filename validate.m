function validate(Kopt, N_tests, Psi, dt, H, noise_params, sample_inits)

	example_metric = zeros(N_tests,1);
    
	for k=1:N_tests
		% get v0 IC for Psi
		v0 = sample_inits();
		% generate data from Psi
		[true_trajectory, observed_trajectory] = data(Psi, H, noise_params, dt, v0);

		% get m0 IC for 3DVAR
		m0 = sample_inits();
		[m_assim, m_pred] = Full3DVAR(m0, Kopt, Psi, observed_trajectory, H, dt);

		% compare assim vs TRUE
		example_metric(k) = mean((m_assim - true_trajectory).^2);

	% Make Plots
    plot(1:N_tests, example_metric, 'xr');
    xlabel('# Test')
    ylabel('Mean Squared Error')
end