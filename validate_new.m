function validate(validate_mode,Kopt, N_tests, Psi, dt, t0, tf,t_future, H, noise_params, sample_inits, is_driven)


% mode=1: how fast below measurement noise
% mode=2: asymptotic running average
% mode=3: predicting the observed statem
% mode=4: error in predicting the full state
% mode=5: how long can we predict in the future up to a specific error
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

example_metric = zeros(N_tests,1);
errorOnTrajectory = zeros(length(t0:dt:tf),size(H,2),N_tests);
evaluateK = zeros(N_tests,1);
for k=1:N_tests
    % get v0 IC for Psi
    v0 = sample_inits();
    % generate data from Psi
     
    [true_trajectory, observed_trajectory, time_whole, drivers_whole] = generateData(Psi, H, noise_params, dt, t0, t_future, v0, is_driven);
        time_up_to_now= t0:dt:tf;
        % true_trajectory = zeros(d, length(time));
    % get m0 IC for 3DVAR
    truncated_true_trajectory=true_trajectory(:,1:length(time_up_to_now));
    truncated_observed_trajectory=observed_trajectory(:,1:length(time_up_to_now));
    m0 = sample_inits();
    [m_assim, m_pred] = Full3DVAR(m0, Kopt, Psi, truncated_observed_trajectory, H, dt, drivers);
    %note that the assim and pred data is up to tf, but not t_future

    % compare assim vs TRUE
    errorOnTrajectory(:,:,k) = (m_assim -  truncated_true_trajectory').^2;
    
    % asymptotic behavior
    asym_threshold=noiseparameters;%need to define the threshold here
        thiserror=0;
        current_time=tf;
        current_pointer=length(time_up_to_now);
        while thiserror<asym_threshold
            current_pointer=current_pointer-1;
            current_time=current_time-dt;
            thiserror=norm(errorOnTrajectory(:,current_pointer,k));
        end
         converge_time=current_time;
         converge_index=current_pointer;
         
     evaluateK(k) = evalK(Kopt, m0, truncated_observed_trajectory, dt, Psi, H, drivers);
    
    if validate_mode==1
        % mode=1: error in predicting the full state.
    example_metric(k) = mean(mean((m_assim -  truncated_true_trajectory').^2));
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if validate_mode==2
        % mode=2: how fast below measurement noise
    example_metric(k) = converge_time;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if validate_mode==3
        % mode=3: asymptotic running average
        m_assim_converge=m_assim(:,converge_index:length(time_up_to_now));
        truncated_true_trajectory_converge=truncated_true_trajectory(:,converge_index:length(time_up_to_now));
       example_metric(k) = mean(mean((m_assim_converge -  truncated_true_trajectory_converge').^2));
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if validate_mode==4
        % mode=4: error in predicting the the observed state
        example_metric(k) =evaluateK(k);
    end
    if validate_mode==5
        % mode=5: how long can we predict in the future up to a specific error
        future_pred_threshold=noise_parameters;%to be defined
        [predicted_future_trajectory, ~, time_future, drivers_future] = generateData(Psi, H, noise_params, dt, tf, t_future,  m_assim(:,length(time_up_to_now)), is_driven);
        true_future_trajectory=true_trajectory(:,length(time_up_to_now),length(time_whole));
        future_pointer=1;
        future_error=0;
        future_time=dt;
      
        while future_error<future_pred_threshold
            future_pointer=future_pointer+1;
            future_time=future_time+dt;
            future_error=norm(true_future_trajectory(:,future_pointer)-predicted_future_trajectory(:,future_pointer));
        end
              example_metric(k)=future_time;
        
    end
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
sigma = sqrt(noise_params.obs_noise.covariance);
for i=1:n
    subplot(n,1,i)
    plot([0,N*dt], [sigma, sigma], '--'); hold on;
    plot(dt*(1:N),errorOnTrajectoryMean(:,i));
    plot(dt*(1:N),errorOnTrajectoryMean(:,i)+errorOnTrajectoryStd(:,i));
    plot(dt*(1:N),max(0,errorOnTrajectoryMean(:,i)-errorOnTrajectoryStd(:,i)));
    set(gca, 'YScale', 'log')
end

end
 
 