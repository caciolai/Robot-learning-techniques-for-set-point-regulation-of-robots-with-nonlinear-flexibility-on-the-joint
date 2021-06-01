%% MPC SIMULATION
clear all;
close all;
clc;

addpath(genpath('../'));
addpath(genpath('./dataGeneration'));
addpath(genpath('./modelFunctions'));
addpath(genpath('./modelsTraining'));
addpath(genpath('./mpcFunctions'));
addpath(genpath('./savedData'));
addpath(genpath('./utils'));

%% Load parameters, model and setup mpc
% robotModel;
mpcSetup;

x0 = params.x0;
u0 = params.u0;
T = params.T;
Ts = params.Ts;
x_ref = params.x_ref;

nx = nlmpcObj.Dimensions.NumberOfStates;
nu = nlmpcObj.Dimensions.NumberOfInputs;
ny = nlmpcObj.Dimensions.NumberOfOutputs;

p = params.controlHorizon;

B = params.B;
D = params.D;

%% Initialize simulation
mv = u0;
xk = x0;

q_ref = x_ref(1:2)';
theta_ref = x_ref(3:4)';

nSamples = T/Ts;
% so that it is a multiple of p
% nSamples = nSamples + (p - mod(nSamples, p)); 

% to record history of simulation
xHistory = zeros(nSamples,nx);
uHistory = zeros(nSamples,nu);

nloptions = nlmpcmoveopt;

%% Simulate nominal system (prendiamo tutti i punti)
for ct = 1:(nSamples/p)
    tau_g = g(q_ref);
%     fprintf("t = %.4f\n", ct * Ts * p);
%     state = xk(1:4)'
    
    [mv,nloptions,info] = nlmpcmove(nlmpcObj,xk,mv);

    for k=1:p
        t = (ct-1)*p + k;
        xk = info.Xopt(k,:)';
        uk = info.MVopt(k,:)';
        
%         tau = tau_g + uk;
%         xk = stateFunctionDT(xk, tau, params);

        xHistory(t,:) = xk';
        uHistory(t,:) = uk';        

    end
    
    mv = info.MVopt(end,:)';
    xk = info.Xopt(end,:)';
end
xHistory(end,:) = xk';
uHistory(end,:) = mv';

disp("Reconstructing model error...");

%% Reconstruct learning history
dataset = params.dataset;
params.model = gpTrain(dataset);
training = true;
theta_dot_old = [0;0];

p = 10;
eHistory = zeros(nSamples/p, 1); % norm of error
for ct = 1:(nSamples/p)
    for k=1:p
        t = (ct-1)*p + k;
        xk = xHistory(t, :)';
        uk = uHistory(t,:)';  
        tau = tau_g + uk;
        
        % Reconstruct elasticity
        if size(dataset,2) < params.datasetDimension
            q = xk(1:2);
            theta = xk(3:4);
            theta_dot = xk(7:8);
            if t > 1
                theta_dot_old = xHistory(t-1, 7:8)';
            end
            theta_ddot = (theta_dot - theta_dot_old)/Ts;
            psi = B*theta_ddot + D*theta_dot - tau;

            dataset(:, end+1) = [q-theta; psi];
        else
            training = false;
        end
    end
    if training
        % Retrain GP on the augmented dataset
        fprintf("Dataset dimension: %d\n", size(dataset, 2));
        disp("Training...");
        tic
        params.model = gpTrain(dataset);
        toc
    end
    
    % Compute predicted and ground truth elastic term
    psi_real = nonlinearElasticity(q-theta, params);
    psi_pred = gpPredict(xk, params.model);
    error = psi_real-psi_pred;
    
    % Compute error norm and update errorHistory
    errorNorm = norm(error);
    eHistory(ct, :) = errorNorm;
end

%% Simulazione "botta unica"
% tic
% [mv,nloptions,info] = nlmpcmove(nlmpcObj,xk,mv);
% toc
% xHistory = info.Xopt;
% uHistory = info.MVopt;

%% Simulate closed-loop system (simulazione vera: solo il primo punto)
% % dataset = params.dataset;
% for ct = 1:nSamples
%     tau_g = g(q_ref);
%     fprintf("t = %.4f\n", ct * Ts);
%     state = xk(1:4)'
%     
%     [mv,nloptions,info] = nlmpcmove(nlmpcObj,xk,mv);
%     tau = tau_g + mv;
%     
%     xk = stateFunctionDT(xk, tau, params);
%     
%     xHistory(ct,:) = xk';
%     uHistory(ct,:) = mv';
% end

%% Plot closed-loop response

t = linspace(0, T, nSamples)';
% t = linspace(0, p, p+1);

figure
subplot(2,1,1)
hold on
grid on
plot(t,xHistory(:,1));
plot(t,repmat(q_ref(1), length(t),1));
xlabel('[s]')
ylabel('[rad]')
legend('$q_1$', '$q_1^d$', 'Interpreter', 'latex', 'Location', 'best');
title('First link position')
set(findall(gcf,'type','line'),'linewidth',2); % Lanari loves it

subplot(2,1,2)
hold on
grid on
plot(t,xHistory(:,2));
plot(t,repmat(q_ref(2), length(t),1));
xlabel('[s]')
ylabel('[rad]')
legend('$q_2$', '$q^d_2$', 'Interpreter', 'latex', 'Location', 'best');
title('Second link position')
set(findall(gcf,'type','line'),'linewidth',2); % Lanari loves it

figure
subplot(2,1,1)
hold on
grid on
plot(t,xHistory(:,1));
plot(t,xHistory(:,3));
xlabel('[s]')
ylabel('[rad]')
legend('$q_1$', '$\theta_1$', 'Interpreter', 'latex', 'Location', 'best');
title('First link and motor position')
set(findall(gcf,'type','line'),'linewidth',2); % Lanari loves it

subplot(2,1,2)
hold on
grid on
plot(t,xHistory(:,2));
plot(t,xHistory(:,4));
xlabel('[s]')
ylabel('[rad]')
legend('$q_2$', '$\theta_2$', 'Interpreter', 'latex', 'Location', 'best');
title('Second link and motor position')
set(findall(gcf,'type','line'),'linewidth',2); % Lanari loves it

% subplot(2,2,3)
% hold on
% grid on
% plot(t,xHistory(:,5))
% plot(t,xHistory(:,6))
% xlabel('[s]')
% ylabel('[rad/s]')
% legend('$\dot{q}_1$', '$\dot{q}_2$', 'Interpreter', 'latex');
% title('Link velocities')

figure
hold on
grid on
plot(t,uHistory(:,1))
plot(t,uHistory(:,2))
xlabel('[s]')
ylabel('[Nm]')
legend('$\tau_1$', '$\tau_2$', 'Interpreter', 'latex', 'Location', 'best');
title('Controlled torque')
set(findall(gcf,'type','line'),'linewidth',2); % Lanari loves it

% plot errore
figure
hold on
grid on
t = linspace(0,10, size(eHistory, 1));
plot(t,eHistory(:,1));
xlabel('Re-training step')
legend('error', 'Location', 'best');
title('Norm of prediction error')
set(findall(gcf,'type','line'),'linewidth',2); % Lanari loves it