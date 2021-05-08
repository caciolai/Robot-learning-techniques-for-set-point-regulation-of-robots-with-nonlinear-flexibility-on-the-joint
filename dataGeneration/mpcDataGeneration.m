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

%% Data manipulation
% Clear old data_mpc
ask = input('Delete old data_mpc? (1 = yes; 0 = no');
if ask == 1
    load('dataNull.mat');
    data_mpc = [input; output];
    save('..\savedData\data_mpc.mat','input','output');
end

%% Load parameters, model and setup mpc
% robotModel;
mpcDataGenSetup;

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

nSamples = T/Ts;

%% Simulate closed-loop system
for ct = 1:nSamples
    tau_g = g(q_ref);
    
    [mv,nloptions,info] = nlmpcmove(nlmpcObj,xk,mv,x_ref);
    tau = tau_g + mv;
    
    xk = stateFunctionDT(xk, tau, params);

%     xHistory(ct,:) = xk';
%     uHistory(ct,:) = mv';
end

%% Plot closed-loop response
% figure
% 
% t = linspace(0, T, nSamples);
% % t = linspace(0, p, p+1);
% 
% subplot(2,2,1)
% hold on
% grid on
% plot(t,xHistory(:,1))
% yline(q_ref(1), '-');
% xlabel('time')
% ylabel('$q_1$', 'Interpreter', 'latex')
% legend('$q_1$', '$q_1^d$', 'Interpreter', 'latex');
% title('First link position')
% 
% subplot(2,2,2)
% hold on
% grid on
% plot(t,xHistory(:,2))
% yline(q_ref(2), '-');
% xlabel('time')
% ylabel('$q_2$', 'Interpreter', 'latex')
% legend('$q_2$', '$q^d_2$', 'Interpreter', 'latex');
% title('Second link position')
% 
% subplot(2,2,3)
% hold on
% grid on
% % plot(t,xHistory(:,5))
% % plot(t,xHistory(:,6))
% % xlabel('time')
% % ylabel('$\dot{q}$', 'Interpreter', 'latex')
% % legend('$\dot{q}_1$', '$\dot{q}_2$', 'Interpreter', 'latex');
% % title('Link velocities')
% plot(t,psiHistory(:,1)-psiGP(:,1))
% plot(t,psiHistory(:,2)-psiGP(:,2))
% xlabel('time')
% ylabel('$\psi^{real} - \psi^{pred}$','Interpreter', 'latex')
% legend('$\psi^{err}_1$','$\psi^{err}_2$','Interpreter', 'latex');
% title('Elasticity prediction error (GP)')
% 
% subplot(2,2,4)
% hold on
% grid on
% % plot(t,uHistory(:,1))
% % plot(t,uHistory(:,2))
% % xlabel('time')
% % ylabel('$\tau$', 'Interpreter', 'latex')
% % legend('$\tau_1$', '$\tau_2$', 'Interpreter', 'latex');
% % title('Controlled torque')
% plot(t,psiHistory(:,1)-psiNN(:,1))
% plot(t,psiHistory(:,2)-psiNN(:,2))
% ylabel('$\psi^{real} - \psi^{pred}$','Interpreter', 'latex')
% legend('$\psi^{err}_1$','$\psi^{err}_2$','Interpreter', 'latex');
% title('Elasticity prediction error (NN)')