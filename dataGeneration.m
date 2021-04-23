%% GENERATION OF DATA TO TRAIN OFFLINE THE GP ON THE ELASTIC TERM
clear all;
close all;
clc

addpath(genpath('./utils'));
addpath(genpath('./modelFunctions'));

%% Parameters 

% Coefficients for motor equation
nLinks = 2;
% rg = 50; % Gear reduction ratio
% Im_zz = 0.015; % Motor inertia 
% B = diag([rg.^2 .* Im_zz, rg.^2 .* Im_zz]);  % Theta inertia matrix

B = eye(2);

% Damping
D = eye(2) * 10; 

% Nonlinear elasticity: k1(q-theta)+k2(q-theta)^3
k1=1e3; 
k2=10;

% Gains for PD to generate data
Kp=diag([40,40]);
Kd=Kp/10;

% Time
T = 1;      % Final time instant
Ts = 1e-3;  % Integration step                   

plotting = false;

%% DATA GENERATION for multiple trajectories

% Planning smooth trajectories for multiple random initial and final
nTrajectories = 10;
q_in = 2*pi*(rand(2, nTrajectories)) - pi*ones(2, nTrajectories);
q_fin = 2*pi*(rand(2, nTrajectories)) - pi*ones(2, nTrajectories);

% Reduction step to undersample trajectories
reductionStep = 10;
nSamplesFull = T*(1/Ts);
nSamples = nSamplesFull/reductionStep;

output_full=zeros(nLinks, nSamplesFull*nTrajectories);
input_full=zeros(2*nLinks, nSamplesFull*nTrajectories);

output = zeros(nLinks, nSamples*nTrajectories);
input = zeros(2*nLinks, nSamples*nTrajectories);

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
    
    [qd, qd_dot,qd_ddot] = get_Trajectory_Desired(q0, q0_dot, q0_ddot, qf, qf_dot, qf_ddot,T,Ts,nLinks);
    
    data = data_gen_single_trajectory(q0, q0_dot,  qd, qd_dot, qd_ddot, T, Ts, nLinks, Kp, Kd, B, k1, k2, D);
    
    x = data(1:4,:); % [q, theta]
    y = data(5:6,:); % elastic terms
    y_real = data(7:8,:);
    
    trajStartStep = (i-1)*(T*(1/Ts))+1;
    trajEndStep = i*(T*(1/Ts));
    
    % Record trajectory
    input_full(:,trajStartStep:trajEndStep) = x;
    output_full(:, trajStartStep:trajEndStep) = y;
    
    % Undersampling
    for j=((i-1)*(T*(1/Ts))+1):reductionStep:(i*(T*(1/Ts)))
        input(:, k) = input_full(:, j);
        output(:, k) = output_full(:, j);
        k = k + 1;
    end
    
    % Plotting
    if plotting
        figure(i)
        hold on
        grid on
        plot(qd(1, :));
        plot(qd(2, :));

        plot(x(1, :));
        plot(x(2, :));

        set(findall(gcf,'type','line'),'linewidth',2); % Lanari loves it

        legend('qd1', 'qd2', 'q1', 'q2');
    end
end
close(bar);

%% Cleanup
close all;
save('data.mat','input','output');
    