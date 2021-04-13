clear all
close all
clc

addpath(genpath('../Functions'));


%% Model parameters
rg = 50; % Gear reduction ratio
Im_zz = 0.015; % Motor inertia 
B = diag([rg.^2 .* Im_zz, rg.^2 .* Im_zz]);  % Theta inertia matrix

% Nonlinear elasticity: k1(q-theta)+k2(q-theta)^3
k1=2.5e2; 
k2=1e2;
K=[k1, 0; 0, k1];

%% Create nonlinear MPC object

nx = 8; % number of state variables (q, theta, q_dot, theta_dot) 
nu = 2; % number of input variables (torques)
ny = 2; % number of output variables (q)

nlobj = nlmpc(nx,ny,nu);

%% MPC parameters
nlobj.Model.IsContinuousTime = false;
nlobj.Model.NumberOfParameters = 4; % (Ts, B, k1, k2)

Ts = 1e-1; %[s]                      % integration step
p = 10;
nlobj.Ts = Ts;
nlobj.PredictionHorizon = p;       
nlobj.ControlHorizon = p;           

%% MPC model

nlobj.Model.StateFcn = "state_function_DT"; % prediction model
nlobj.Model.OutputFcn = @(x, u, Ts, B, k1, k2) [x(1); x(2)]; % y = [q1, q2];

%% MPC cost
% Custom Cost function
mpc.Optimization.CustomCostFcn = @(x, u, e, mpc) cost_mpc(X,U,e,mpc);
mpc.Optimization.ReplaceStandardCost = true;


%% MPC constraints

% Limited torque capability
maxTorque = 100;
nlobj.MV(1).Min = -maxTorque;
nlobj.MV(1).Max = maxTorque;
nlobj.MV(2).Min = -maxTorque;
nlobj.MV(2).Max = maxTorque;

%% Validate model
q0 = [0; 0];
theta0 = q0;
q0_dot = [0; 0];
theta0_dot = q0_dot;
params = {Ts, B, k1, k2};

x0 = [q0; theta0; q0_dot; theta0_dot];
u0 = [0; 0];
validateFcns(nlobj,x0,u0,[],params);

%% Initialize simulation
nloptions = nlmpcmoveopt;
nloptions.Parameters = params;
mv = [0; 0]; %  optimal control move computed at any control interval

xk = x0;
T = 10; % [s]
xHistory = zeros(nx, T/Ts);
xHistory(:, 1) = x0;
uHistory = zeros(nu, T/Ts);
uHistory(:, 1) = u0;

%% Setpoint references

% Upright position
q_ref = [pi/2 pi/2]; % qd
theta_ref =  q_ref + K\g(q_ref);

%% Simulate closed-loop system
hbar = waitbar(0,'Simulation Progress');
for ct = 1:(T/Ts)
    % Compute optimal control moves.
    [mv,nloptions,info] = nlmpcmove(nlobj,xk,mv,q_ref,[],nloptions);
    
    % Implement first optimal control move and update plant states.
    xk = state_function_DT(xk, mv, Ts, B, k1, k2);
    
    % Save plant states for display.
    xHistory(:, ct) = xk;
    uHistory(:, ct) = mv;
    waitbar(ct*Ts/T,hbar);
end
close(hbar)

%% Plot closed-loop response
figure

t = linspace(0, T, T/Ts);

subplot(2,2,1)
hold on
plot(t,xHistory(1,:))
yline(q_ref(1), 'r-');
xlabel('time')
ylabel('q1')
legend('q1', 'qd1');
title('First link position')

subplot(2,2,2)
hold on
plot(t,xHistory(2,:))
yline(q_ref(2), 'r-');
xlabel('time')
ylabel('q2')
legend('q2', 'qd2');
title('Second link position')

subplot(2,2,3)
hold on
plot(t,xHistory(3,:))
plot(t,xHistory(4,:))
xlabel('time')
ylabel('q_{dot}')
legend('q_{dot}_1', 'q_{dot}_2');
title('Link velocities')

subplot(2,2,4)
hold on
plot(t,uHistory(1,:))
plot(t,uHistory(2,:))
xlabel('time')
ylabel('tau')
legend('tau1', 'tau2');
title('Controlled torque')