%% MPC SIMULATION
clear all;
close all;
clc;

addpath(genpath('./utils'));
addpath(genpath('./modelFunctions'));
addpath(genpath('./mpcFunctions'));

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

p = params.p;

%% Initialize simulation
mv = u0;
xk = x0;

q_ref = x_ref(1:2)';
theta_ref = x_ref(3:4)';

% to record history of simulation
nSamples = T/Ts;
xHistory = zeros(nSamples,nx);
uHistory = zeros(nSamples,nu);
psiHistory = zeros(nSamples,2);
psiPredicted = zeros(nSamples,2);

%% Simulate closed-loop system
for ct = 1:nSamples
    tau_g = g(q_ref);
    fprintf("t = %.4f\n", (ct-1)*Ts);
    fprintf("x = %.4f\t%.4f\t%.4f\t%.4f\n", xk(1),xk(2),xk(3),xk(4));
    
    [mv,nloptions,info] = nlmpcmove(nlmpcObj,xk,mv,x_ref);
    tau = tau_g + mv;
    
    psiHistory(ct,:) = nonlinearElasticity(xk, params.K1, params.K2);
    psiPredicted(ct,:) = gpPredict(xk, params.model);
    
    xk = stateFunctionDT(xk, tau, params);
    
    xHistory(ct,:) = xk';
    uHistory(ct,:) = mv';
end

% tic
% [mv,nloptions,info] = nlmpcmove(nlmpcObj,xk,mv);
% toc
% xHistory = info.Xopt;
% uHistory = info.MVopt;

%% Plot closed-loop response
figure

% t = linspace(0, T, nSamples);
t = linspace(0, p, p+1);

subplot(2,2,1)
hold on
grid on
plot(t,xHistory(:,1))
yline(q_ref(1), 'r-');
xlabel('time')
ylabel('$q_1$', 'Interpreter', 'latex')
legend('$q_1$', '$q_1^d$', 'Interpreter', 'latex');
title('First link position')

subplot(2,2,2)
hold on
grid on
plot(t,xHistory(:,2))
yline(q_ref(2), 'r-');
xlabel('time')
ylabel('$q_2$', 'Interpreter', 'latex')
legend('$q_2$', '$q^d_2$', 'Interpreter', 'latex');
title('Second link position')

subplot(2,2,3)
hold on
grid on
% plot(t,xHistory(:,5))
% plot(t,xHistory(:,6))
% xlabel('time')
% ylabel('$\dot{q}$', 'Interpreter', 'latex')
% legend('$\dot{q}_1$', '$\dot{q}_2$', 'Interpreter', 'latex');
% title('Link velocities')
plot(t,psiHistory(:,1))
plot(t,psiHistory(:,2))
xlabel('time')
ylabel('$\psi$', 'Interpreter', 'latex')
legend('$\psi^{pred}_1$', '$\psi^{pred}_2$', ...
    '\psi^{pred}_3','\psi^{pred}_4',... 
    'Interpreter', 'latex');
title('Elasticity')

subplot(2,2,4)
hold on
grid on
plot(t,uHistory(:,1))
plot(t,uHistory(:,2))
xlabel('time')
ylabel('$\tau$', 'Interpreter', 'latex')
legend('$\tau_1$', '$\tau_2$', 'Interpreter', 'latex');
title('Controlled torque')