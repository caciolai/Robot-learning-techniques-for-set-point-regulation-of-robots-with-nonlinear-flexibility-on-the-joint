%% MPC SIMULATION
clear all;
close all;
clc;

addpath(genpath('./utils'));
addpath(genpath('./modelFunctions'));
addpath(genpath('./mpcFunctions'));

%% Load model and setup mpc
% robotModel;
mpcSetup;

%% Initialize simulation
mv = u0;
xk = x0;

q_ref = x_ref(1:2)';
theta_ref = x_ref(3:4)';

% to record history of simulation
xHistory = zeros(nx, T/Ts);
xHistory(:,1) = x0;
uHistory = zeros(nu, T/Ts);
uHistory(:,1) = u0;

%% Simulate closed-loop system
% hbar = waitbar(0,'Simulation Progress');
% for ct = 1:(T/Ts)
%     tau_g = g(q_ref);
%     fprintf("t = %.4f\n", (ct-1)*Ts);
%     disp(xk');
%     
%     % Compute optimal control moves.
%     [mv,nloptions,info] = nlmpcmove(nlmpcObj,xk,mv,x_ref,[],nloptions);
%     
%     tau = tau_g + mv;
%     
%     % Implement first optimal control move and update plant states.
%     xk = stateFunctionDT(xk, tau, Ts, B, K1, K2, D);
%     
%     % Save plant states for display.
%     xHistory(ct,:) = xk';
%     uHistory(ct,:) = tau';
% %     waitbar(ct*Ts/T);
% end
% % close(hbar);

tau_g = g(q_ref);
[mv,nloptions,info] = nlmpcmove(nlmpcObj,xk,mv,x_ref,[],nloptions);
xHistory = info.Xopt;
uHistory = info.MVopt;

%% Plot closed-loop response
figure

% t = linspace(0, T, T/Ts);
t = linspace(1, p+1, p+1);

subplot(2,2,1)
hold on
plot(t,xHistory(:,1))
yline(q_ref(1), 'r-');
xlabel('time')
ylabel('q1')
legend('q1', 'qd1');
title('First link position')

subplot(2,2,2)
hold on
plot(t,xHistory(:,2))
yline(q_ref(2), 'r-');
xlabel('time')
ylabel('q2')
legend('q2', 'qd2');
title('Second link position')

subplot(2,2,3)
hold on
plot(t,xHistory(:,3))
plot(t,xHistory(:,4))
xlabel('time')
ylabel('q_{dot}')
legend('q_{dot}_1', 'q_{dot}_2');
title('Link velocities')

subplot(2,2,4)
hold on
plot(t,uHistory(:,1))
plot(t,uHistory(:,2))
xlabel('time')
ylabel('tau')
legend('tau1', 'tau2');
title('Controlled torque')