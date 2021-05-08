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
psiGP = zeros(nSamples,2);
psiNN = zeros(nSamples,2);

%% Simulate nominal system
% for ct = 1:(nSamples/params.p)
%     tau_g = g(q_ref);
%     fprintf("t = %.4f\n", (ct-1)*Ts*params.p);
%     fprintf("x = %.4f\t%.4f\t%.4f\t%.4f\n", xk(1),xk(2),xk(3),xk(4));
%     
%     [mv,nloptions,info] = nlmpcmove(nlmpcObj,xk,mv,x_ref);
% %     tau = tau_g + mv;
%     for k=1:params.p
%         t = (ct-1)*params.p + k;
%         xk = info.Xopt(k,:)';
%         uk = info.MVopt(k,:)';
%         
%         psiHistory(t,:) = nonlinearElasticity(xk, params.K1, params.K2);
%         psiGP(t,:) = gpPredict(xk, params.model);
%         psiNN(t,:) = nnMdl(xk(1:4));
% %         xk = stateFunctionDT(xk, tau, params);
% 
%         xHistory(t,:) = xk';
%         uHistory(t,:) = mv';
%     end
%     mv = info.MVopt(end,:)';
%     xk = info.Xopt(end,:)';
% end

tic
[mv,nloptions,info] = nlmpcmove(nlmpcObj,xk,mv);
toc
xHistory = info.Xopt;
uHistory = info.MVopt;

% %% Simulate closed-loop system
% for ct = 1:nSamples
%     tau_g = g(q_ref);
%     fprintf("t = %.4f\n", (ct-1)*Ts);
%     fprintf("x = %.4f\t%.4f\t%.4f\t%.4f\n", xk(1),xk(2),xk(3),xk(4));
%     
%     [mv,nloptions,info] = nlmpcmove(nlmpcObj,xk,mv,x_ref);
%     tau = tau_g + mv;
%         
%     psiHistory(ct,:) = nonlinearElasticity(xk, params.K1, params.K2);
%     psiGP(ct,:) = gpPredict(xk, params.model);
%     psiNN(ct,:) = nnMdl(xk(1:4));
%     
%     xk = stateFunctionDT(xk, tau, params);
% 
%     xHistory(ct,:) = xk';
%     uHistory(ct,:) = mv';
% end

%% Plot closed-loop response
figure

% t = linspace(0, T, nSamples);
t = linspace(0, p, p+1);

subplot(2,2,1)
hold on
grid on
plot(t,xHistory(:,1))
yline(q_ref(1), '-');
xlabel('time')
ylabel('$q_1$', 'Interpreter', 'latex')
legend('$q_1$', '$q_1^d$', 'Interpreter', 'latex');
title('First link position')

subplot(2,2,2)
hold on
grid on
plot(t,xHistory(:,2))
yline(q_ref(2), '-');
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
plot(t,psiHistory(:,1)-psiGP(:,1))
plot(t,psiHistory(:,2)-psiGP(:,2))
xlabel('time')
ylabel('$\psi^{real} - \psi^{pred}$','Interpreter', 'latex')
legend('$\psi^{err}_1$','$\psi^{err}_2$','Interpreter', 'latex');
title('Elasticity prediction error (GP)')

subplot(2,2,4)
hold on
grid on
% plot(t,uHistory(:,1))
% plot(t,uHistory(:,2))
% xlabel('time')
% ylabel('$\tau$', 'Interpreter', 'latex')
% legend('$\tau_1$', '$\tau_2$', 'Interpreter', 'latex');
% title('Controlled torque')
plot(t,psiHistory(:,1)-psiNN(:,1))
plot(t,psiHistory(:,2)-psiNN(:,2))
ylabel('$\psi^{real} - \psi^{pred}$','Interpreter', 'latex')
legend('$\psi^{err}_1$','$\psi^{err}_2$','Interpreter', 'latex');
title('Elasticity prediction error (NN)')