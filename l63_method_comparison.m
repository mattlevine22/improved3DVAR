useEnKF = 0;
manyPlots = 0;

% overall settings
H = [1 0 0];
is_driven = 0;
dt = 0.01;
noise_params = struct();
noise_params.state_noise.mean = 0;
noise_params.state_noise.covariance = 0;
noise_params.obs_noise.mean = 0;
noise_params.obs_noise.covariance = 1;


%training settings
t0 = 0;
tf = 1;
N_training_sets = 2;

% test settings
N_testing_sets = 2;
t0_test = t0;
tf_test = 10;
tf_future = 20;

% algorithm settings
N_runs_per_training_set = 2;
% gd settings
gd_learning_rate = 0.0005;
gd_Kinit_sd = 0.1;

%% Training
% create training data
for j=1:N_training_sets
    v0 = get_lorenz_inits();
    [true_trajectory_tmp, observed_trajectory_tmp, time_tmp, drivers_tmp] = generateData(@PsiL63, H, noise_params, dt, t0, tf, v0, is_driven);
    
    time=time_tmp; % doesn't change per data generation
    drivers=drivers_tmp; % doesn't change per data generation
    true_trajectory(j,:,:) = true_trajectory_tmp;
    observed_trajectory(j,:,:) = observed_trajectory_tmp;
end

% Set and evaluate theory-based gains
K_struct = struct();
K_struct.algorithm.theory0.Kopt(1,1,:) = H/(1+0);
K_struct.algorithm.theory1.Kopt(1,1,:) = H/(1+0.1);
K_struct.algorithm.theory2.Kopt(1,1,:) = H/(1+0.25);

% Run Gradient Descent Algorithms
for j=1:N_training_sets
    for k=1:N_runs_per_training_set
        K0 = gd_Kinit_sd*randn(size(H'));
        m0 = get_lorenz_inits();
        KoptGD = GDfullstates(m0, squeeze(true_trajectory(j,:,:)), squeeze(observed_trajectory(j,:,:))', dt, K0, gd_learning_rate, @PsiL63, H, drivers);
        K_struct.algorithm.GD.Kopt(j,k,:) = KoptGD;
        K_struct.algorithm.GD.eval(j,k) = validate_new(KoptGD, N_testing_sets, @PsiL63, dt, t0_test, tf_test, tf_future, H, noise_params, @get_lorenz_inits, is_driven);
    end
end

% Summarize learned K's in terms of mean/sd
for nm_alg=fields(K_struct.algorithm)'
    nm_alg = char(nm_alg);
    if 1==(size(K_struct.algorithm.(nm_alg).Kopt,1) * size(K_struct.algorithm.(nm_alg).Kopt,2))
        K_struct.algorithm.(nm_alg).K_summary.mean = squeeze(K_struct.algorithm.(nm_alg).Kopt);
        K_struct.algorithm.(nm_alg).K_summary.sd = 0*K_struct.algorithm.(nm_alg).K_summary.mean;    
    else
        K_struct.algorithm.(nm_alg).K_summary.mean = squeeze(mean(K_struct.algorithm.(nm_alg).Kopt,[1,2]));
        K_struct.algorithm.(nm_alg).K_summary.sd = squeeze(std(K_struct.algorithm.(nm_alg).Kopt,[],[1,2]));
    end
        
end

%% Testing
K_struct.evals = struct();
for nm_alg=fields(K_struct.algorithm)'
    nm_alg = char(nm_alg);
    for j=1:size(K_struct.algorithm.(nm_alg).Kopt,1)
        for k=1:size(K_struct.algorithm.(nm_alg).Kopt,2)
            my_K = squeeze(K_struct.algorithm.(nm_alg).Kopt(j,k,:));
            eval_output = validate_new(my_K, N_testing_sets, @PsiL63, dt, t0_test, tf_test, tf_future, H, noise_params, @get_lorenz_inits, is_driven);
            K_struct.algorithm.(nm_alg).eval(j,k) = eval_output;
            for nm_metric=fields(eval_output)'
                nm_metric = char(nm_metric);
                eval_data = eval_output.(nm_metric).data;
                if ~isfield(K_struct.evals,nm_metric) || ~isfield(K_struct.evals.(nm_metric),nm_alg)
                    K_struct.evals.(nm_metric).(nm_alg) = eval_data;
                else
                    K_struct.evals.(nm_metric).(nm_alg) = [K_struct.evals.(nm_metric).(nm_alg); eval_data];
                end
            end
            
            close all;
        end
    end
end

%% Group test results
N_algs = length(fields(K_struct.algorithm));

for nm_metric=fields(K_struct.algorithm.(nm_alg).eval)'
    nm_metric = char(nm_metric);
    K_struct.evals.(nm_metric).grouped_evals = nan(N_training_sets*N_runs_per_training_set*N_testing_sets,N_algs);
    c = 0;
    for nm_alg=fields(K_struct.algorithm)'
        c = c + 1;
        nm_alg = char(nm_alg);
        my_data = K_struct.evals.(nm_metric).(nm_alg);
        K_struct.evals.(nm_metric).grouped_evals(1:length(my_data),c) = my_data;
    end
end

%% Plot test performance
for nm_metric=fields(K_struct.algorithm.(nm_alg).eval)'
    nm_metric = char(nm_metric);
    figure;
    boxplot(K_struct.evals.(nm_metric).grouped_evals,fields(K_struct.algorithm)')
    title(nm_metric)
    xlabel('Algorithm')
    ylabel('Performance')
end

%% Plot K comparison
figure;
K_len = size(K_struct.algorithm.(nm_alg).Kopt,3);

subplot(1,K_len,1); hold on;
xind = 1;
yind = 2;
c = 0;
for nm_alg=fields(K_struct.algorithm)'
    c = c + 1;
    nm_alg = char(nm_alg);
    x_mean = K_struct.algorithm.(nm_alg).K_summary.mean(xind);
    x_sd = K_struct.algorithm.(nm_alg).K_summary.sd(xind);
    y_mean = K_struct.algorithm.(nm_alg).K_summary.mean(yind);
    y_sd = K_struct.algorithm.(nm_alg).K_summary.sd(yind);
    errorbar(x_mean,y_mean,y_sd,y_sd,x_sd,x_sd)
%     errorbarXx,Y,YNEG,YPOS,XNEG,XPOS)
end
xlabel(sprintf('K_%d',xind))
ylabel(sprintf('K_%d',yind))
legend(fields(K_struct.algorithm)')
title(sprintf('K_%d vs K_%d',xind, yind))

%
subplot(1,K_len,2); hold on;
xind = 1;
yind = 3;
c = 0;
for nm_alg=fields(K_struct.algorithm)'
    c = c + 1;
    nm_alg = char(nm_alg);
    x_mean = K_struct.algorithm.(nm_alg).K_summary.mean(xind);
    x_sd = K_struct.algorithm.(nm_alg).K_summary.sd(xind);
    y_mean = K_struct.algorithm.(nm_alg).K_summary.mean(yind);
    y_sd = K_struct.algorithm.(nm_alg).K_summary.sd(yind);
    errorbar(x_mean,y_mean,y_sd,y_sd,x_sd,x_sd)
%     errorbarXx,Y,YNEG,YPOS,XNEG,XPOS)
end
xlabel(sprintf('K_%d',xind))
ylabel(sprintf('K_%d',yind))
legend(fields(K_struct.algorithm)')
title(sprintf('K_%d vs K_%d',xind, yind))

%
subplot(1,K_len,3); hold on;
xind = 2;
yind = 3;
c = 0;
for nm_alg=fields(K_struct.algorithm)'
    c = c + 1;
    nm_alg = char(nm_alg);
    x_mean = K_struct.algorithm.(nm_alg).K_summary.mean(xind);
    x_sd = K_struct.algorithm.(nm_alg).K_summary.sd(xind);
    y_mean = K_struct.algorithm.(nm_alg).K_summary.mean(yind);
    y_sd = K_struct.algorithm.(nm_alg).K_summary.sd(yind);
    errorbar(x_mean,y_mean,y_sd,y_sd,x_sd,x_sd)
%     errorbarXx,Y,YNEG,YPOS,XNEG,XPOS)
end
xlabel(sprintf('K_%d',xind))
ylabel(sprintf('K_%d',yind))
legend(fields(K_struct.algorithm)')
title(sprintf('K_%d vs K_%d',xind, yind))

%%
%example use of EnKF
% if(useEnKF)
%     noise_params_EnKF = noise_params;
%     noise_params_EnKF.state_noise.mean = zeros(length(H),1);
%     noise_params_EnKF.state_noise.covariance = eye(length(H));
%     number_Particles = 50; %how many?
%     v0 = get_lorenz_inits();
%     [est_traj,Kenkf] = EnKF(v0, observed_trajectory, dt, noise_params_EnKF, number_Particles, @PsiL63, H, drivers);
%
%     %visualization of EnKF
%     N=length(observed_trajectory);
%     figure(8)
%     for i=1:3
%         subplot(3,1,i)
%         plot(dt*(1:N),true_trajectory(i,:),'-b'); hold on;
%         plot(dt*(1:N),est_traj(i,:),'--');
%         legend('true', 'est')
%     end
%     subplot(3,1,1)
%     plot(dt*(1:N),observed_trajectory(:),'-g')
%     true_trajectory = est_traj;
% end
%
% if(manyPlots)
%     fig = figure;
%     plot3(true_trajectory(1,:), true_trajectory(2,:),true_trajectory(3,:),'r')
%     title('Full trajectory (ground truth) of L63')
%     legend('full states')
%     xlabel('x')
%     ylabel('y')
%     zlabel('z')
%     saveas(fig,[pwd '/plots/fullTrajectoryL63.png'])
%     savefig(fig, [pwd '/plots/fullTrajectoryL63.fig'])
%
%     fig = figure;
%     plot(time, observed_trajectory, 'b')
%     title('Observed trajectory (partial, noisy) of L63')
%     legend('observed y')
%     xlabel('t')
%     ylabel('y')
%     saveas(fig,[pwd '/plots/observedTrajectoryL63.png'])
%     savefig(fig,[pwd '/plots/observedTrajectoryL63.fig'])
%
% end



tic
%KoptOpt = OptObservations(m0, observed_trajectory, dt, KoptGD, @PsiL63, H, drivers);
disp('For the optimization: ')
toc

%errorKoptOpt = evalK(KoptOpt, m0, observed_trajectory, dt,@PsiL63, H, drivers)

%gainValues(j) = Kopt(1);
%gainValues2(j) = Kopt(2);
% end
%figure();
%plot(1:runs,gainValues, 'xb')
%fig=figure;
%histogram(gainValues)
%hold on;
%m = mean(gainValues);
%plot([m m],[0 runs/2],'-r')
%savefig(fig, 'figname.fig');

% output = validate_new(KoptGD, N_tests, @PsiL63, dt, t0_test, tf_test, tf_future, H, noise_params, @get_lorenz_inits, is_driven);


% Make Plots
figure
histogram(output.metric1.data);
title('error in predicting the full state')

figure
histogram(output.metric2.data);
title('how fast below measurement noise')

figure
histogram(output.metric3.data);
title('asymptotic running average')

figure
histogram(output.metric4.data);
title('error in predicting the observed state')

figure
histogram(output.metric5.data);
title('how long can we predict in the future')