%% MPC PARAMETERS AND OBJECT
clear all
close all
clc

addpath(genpath('./modelFunctions'));
addpath(genpath('./mpcFunctions'));

%% Elastic parameters 
% rg = 50; % Gear reduction ratio
% Im_zz = 0.015; % Motor inertia 
% B = diag([rg.^2 .* Im_zz, rg.^2 .* Im_zz]);  % Theta inertia matrix

% Motor inertia matrix
B = eye(2);

% Stiffness matrices (for elasticity)
K1 = eye(2) * 1000;
K2 = eye(2) * 10;

% Damping
D = eye(2) * 10; 

%% Create nonlinear MPC object

nx = 8; % number of state variables (q, theta, q_dot, theta_dot) 
ny = 8; % number of output variables (full state)
nu = 2; % number of input variables (torques)

nlmpcObj = nlmpc(nx,ny,nu);

%% MPC model

nlmpcObj.Model.StateFcn = "mpcStateFunctionDT"; 
nlmpcObj.Model.OutputFcn = "mpcOutputFunction";

%% MPC cost
nlmpcObj.Optimization.CustomCostFcn = "mpcCostFunction";
nlmpcObj.Optimization.ReplaceStandardCost = true;

u_max = 1000;
nlmpcObj.ManipulatedVariables(1).Min = -u_max;
nlmpcObj.ManipulatedVariables(1).Max = +u_max;
nlmpcObj.ManipulatedVariables(2).Min = -u_max;
nlmpcObj.ManipulatedVariables(2).Max = +u_max;

%% MPC constraints
nlmpcObj.Optimization.CustomIneqConFcn = "mpcInequalityConstraints";

%% Reference

% desired link position
q_ref = [pi/2 pi/4]';

% SET POINT FOR NONLINEAR ELASTICITY
% vogliamo che all'equilibrio il termine elastico compensi la gravità
g_ref = g(q_ref);

prob = eqnproblem;
x = optimvar('x',2);
eqn1 = -K1(1,1) * (q_ref(1) - x(1)) - K2(1,1) * (q_ref(1) - x(1))^3 - g_ref(1) == 0;
eqn2 = -K1(2,2) * (q_ref(2) - x(2)) - K2(2,2) * (q_ref(2) - x(2))^3 - g_ref(2) == 0;
x0.x = [0.0, 0.0];

prob.Equations.eqn1 = eqn1;
prob.Equations.eqn2 = eqn2;

[sol,fval,exitflag] = solve(prob,x0);
theta_ref = sol.x;

% desired equilibrium (no velocity)
x_ref = [q_ref; theta_ref; zeros(4, 1)]'; % must be row vector

%% MPC parameters
Ts = 1e-2;                              % integration step
p = 100;                                 % control/prediction horizon
nlmpcObj.Ts = Ts;
nlmpcObj.PredictionHorizon = p;       
nlmpcObj.ControlHorizon = p;       

params = {Ts, B, K1, K2, D, x_ref, p};
nlmpcObj.Model.IsContinuousTime = false;
nlmpcObj.Model.NumberOfParameters = length(params);

nloptions = nlmpcmoveopt;
nloptions.Parameters = params;

% nlmpcObj.Optimization.SolverOptions.Algorithm ='interior-point'; 
nlmpcObj.Optimization.SolverOptions.MaxIterations = 400;

%% Simulation parameters
% simulation time
T = 10.0; % [s]

% initial configuration
q0 = [0; 0];
theta0 = q0;
q0_dot = [0; 0];
theta0_dot = q0_dot;

x0 = [q0; theta0; q0_dot; theta0_dot];
u0 = [0; 0];

%% Validate model
validateFcns(nlmpcObj,x0,u0,[],params);
