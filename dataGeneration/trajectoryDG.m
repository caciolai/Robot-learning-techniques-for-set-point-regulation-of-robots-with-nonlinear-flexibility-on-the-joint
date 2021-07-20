%% GENERATION OF DATA TO TRAIN OFFLINE THE GP ON THE ELASTIC TERM
clc 

addpath(genpath('../'));
addpath(genpath('./dataGeneration'));
addpath(genpath('./modelFunctions'));
addpath(genpath('./modelsTraining'));
addpath(genpath('./mpcFunctions'));
addpath(genpath('./savedData'));
addpath(genpath('./utils'));

%% Parameters 
parameters;
plotting = false;

reductionStep = params.reductionStep;
nTrajectories = params.nTrajectories;
dgT = params.dgT;
dgTs = params.dgTs;
B = params.B;
K1 = params.K1;
K2 = params.K2;
D = params.D;

%% DATA GENERATION for multiple trajectories

% Planning smooth trajectories for multiple random initial and final
q_in = 2*pi*(rand(2, nTrajectories)) - pi*ones(2, nTrajectories);
q_fin = 2*pi*(rand(2, nTrajectories)) - pi*ones(2, nTrajectories);

if plotting
    figure(1)
    % To check if the configuration space is covered
    hold on
    for i=1:nTrajectories
        plot(q_in(1,i), q_in(2,i), 'o')
    end
    xlim([-pi, pi]);
    ylim([-pi, pi]);

    figure(2)
    % To check if the configuration space is covered
    hold on
    for i=1:nTrajectories
        plot(q_fin(1,i), q_fin(2,i), '+')
    end
    xlim([-pi, pi]);
    ylim([-pi, pi]);
end
    
% Reduction step to undersample trajectories
nSamplesFull = dgT*(1/dgTs);
nSamples = nSamplesFull/reductionStep;

output_full=zeros(nLinks, nSamplesFull*nTrajectories);
input_full=zeros(nLinks, nSamplesFull*nTrajectories);

output = zeros(nLinks, nSamples*nTrajectories);
input = zeros(nLinks, nSamples*nTrajectories);

bar = waitbar(0,'Data generation ...');

k = 1;
for i=1:nTrajectories

    waitbar(i/length(q_in),bar);

    q0      = q_in(:,i);
    q0_dot  = [0;0];
    q0_ddot = [0;0];
    
    qf      = q_fin(:,i);
    qf_dot  = [0;0];
    qf_ddot = [0;0];
    
    [qd, qd_dot,qd_ddot] = get_Trajectory_Desired(q0, q0_dot, q0_ddot, qf, qf_dot, qf_ddot,dgT,dgTs,nLinks);
    
    data = data_gen_single_trajectory(q0, q0_dot, ...
        qd, qd_dot, qd_ddot, params);
    
    x = data(1:2,:); % [theta - q]
    y = data(3:4,:); % elastic terms
    y_real = data(5:6,:); % psi_real
    q = x(1:2,:);
    
    trajStartStep = (i-1)*nSamplesFull+1;
    trajEndStep = i*nSamplesFull;
    
    % Record trajectory
    input_full(:,trajStartStep:trajEndStep) = x;
    output_full(:,trajStartStep:trajEndStep) = y;
    
    % Undersampling
    for j=((i-1)*nSamplesFull+1):reductionStep:(i*nSamplesFull)
        input(:, k) = input_full(:, j);
        output(:, k) = output_full(:, j);
        k = k + 1;
    end
    
    % Plotting
    if plotting
        figure(i)
        subplot(1,2,1);
        hold on
        grid on
        plot(y(1, :));
        plot(y_real(1, :));
        set(findall(gcf,'type','line'),'linewidth',2); % Lanari loves it
        xlabel('time');
        ylabel('$\psi_1$', 'Interpreter', 'latex');
        legend('$y$', '$y_{real}$', 'Interpreter', 'latex');
        title("Elasticity (first joint)");
        
        subplot(1,2,2);
        hold on
        grid on
        plot(y(2, :));
        plot(y_real(2, :));
        set(findall(gcf,'type','line'),'linewidth',2); % Lanari loves it
        xlabel('time');
        ylabel('$\psi_1$', 'Interpreter', 'latex');
        legend('$y$', '$y_{real}$', 'Interpreter', 'latex');
        title("Elasticity (second joint)");
        
    end
end
close(bar);

%% Cleanup
close all;
save('..\savedData\data_traj.mat','input','output');
clear all;
    