function validate(Kopt, N_tests, Psi, dt, H, noise_params, sample_inits)

t0 = 0;
tf = 10;
example_metric = zeros(N_tests,1);

for k=1:N_tests
    % get v0 IC for Psi
    v0 = sample_inits();
    % generate data from Psi
    [true_trajectory, observed_trajectory] = generateData(Psi, H, noise_params, dt, t0, tf, v0);
    
    % get m0 IC for 3DVAR
    m0 = sample_inits();
    [m_assim, m_pred] = Full3DVAR(m0, Kopt, Psi, observed_trajectory, H, dt);
    
    % compare assim vs TRUE
    example_metric(k) = mean(mean((m_assim - true_trajectory').^2));
end

% Make Plots
figure;
plot(1:N_tests, example_metric, 'xr');
xlabel('# Test')
ylabel('Mean Squared Error')

figure;
M = length(m0);
N = length(true_trajectory);
for i=1:M
    subplot(M,1,i)
    plot(dt*(1:N),m_assim(:,i)); hold on;
    plot(dt*(1:N),true_trajectory(i,:));
end

end