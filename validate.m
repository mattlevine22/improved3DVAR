function validate(Kopt, N_tests, Psi, dt, H, noise_params, sample_inits)
close all;
t0 = 0;
tf = 10;
example_metric = zeros(N_tests,1);
errorOnTrajectory = zeros(length(t0:dt:tf),size(H,2),N_tests);
evaluateK = zeros(N_tests,1);
for k=1:N_tests
    % get v0 IC for Psi
    v0 = sample_inits();
    % generate data from Psi
    [true_trajectory, observed_trajectory] = generateData(Psi, H, noise_params, dt, t0, tf, v0);
    
    % get m0 IC for 3DVAR
    m0 = sample_inits();
    [m_assim, m_pred] = Full3DVAR(m0, Kopt, Psi, observed_trajectory, H, dt);
    
    % compare assim vs TRUE
    errorOnTrajectory(:,:,k) = (m_assim - true_trajectory').^2;
    example_metric(k) = mean(mean((m_assim - true_trajectory').^2));
    evaluateK(k) = evalK(Kopt, m0, observed_trajectory, dt,Psi, H);
end

errorOnTrajectoryMean = mean(errorOnTrajectory, 3);
errorOnTrajectoryStd = std(errorOnTrajectory, 0, 3);
% Make Plots
%figure;
%plot(1:N_tests, example_metric, 'xr');
%xlabel('# Test')
%ylabel('Mean Squared Error')
Kopt
example_metric
evaluateK

figure;
M = length(m0);
N = length(true_trajectory);
for i=1:M
    subplot(M,1,i)
    plot(dt*(1:N),m_assim(:,i)); hold on;
    plot(dt*(1:N),true_trajectory(i,:),'--');
end

figure;
n = size(H,2);
N = size(errorOnTrajectoryMean,1);
sigma = sqrt(noise_params.variance);
for i=1:n
    subplot(n,1,i)
    plot([0,N*dt], [sigma, sigma], '--'); hold on;
    plot(dt*(1:N),errorOnTrajectoryMean(:,i));
    plot(dt*(1:N),errorOnTrajectoryMean(:,i)+errorOnTrajectoryStd(:,i));
    plot(dt*(1:N),max(0,errorOnTrajectoryMean(:,i)-errorOnTrajectoryStd(:,i)));
    set(gca, 'YScale', 'log')
end

end