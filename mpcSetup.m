%% MPC PARAMETERS AND OBJECT
clear all
close all
clc

addpath(genpath('./modelFunctions'));
addpath(genpath('./mpcFunctions'));

%% Model parameters
rg = 50; % Gear reduction ratio
Im_zz = 0.015; % Motor inertia 
B = diag([rg.^2 .* Im_zz, rg.^2 .* Im_zz]);  % Theta inertia matrix

% Elasticity 
K1 = eye(2) * 1000;
K2 = eye(2) * 10;

%% Create nonlinear MPC object

nx = 8; % number of state variables (q, theta, q_dot, theta_dot) 
ny = 8; % number of output variables (full state)
nu = 2; % number of input variables (torques)

nlobj = nlmpc(nx,ny,nu);

%% MPC model

% prediction model
nlobj.Model.StateFcn = "mpcStateFunctionDT"; 

nlobj.Model.OutputFcn = "mpcOutputFunction";

%% MPC cost
% Custom Cost function
mpc.Optimization.CustomCostFcn = "mpcCostFunction";
mpc.Optimization.ReplaceStandardCost = true;


%% MPC constraints
mpc.Optimization.CustomIneqConFcn = "mpcInequalityConstraints";

%% Reference

% desired link position
q_ref = [pi/2 pi/2]';

% vogliamo che all'equilibrio il termine elastico compensi la gravità
% NB: assumiamo che il termine lineare domini per q-theta molto piccoli
theta_ref = q_ref + K1\g(q_ref);

% desired equilibrium (no velocity)
x_ref = [q_ref; theta_ref; zeros(4, 1)]'; % must be row vector

%% MPC parameters
Ts = 1e-2;                              % integration step
p = 10;                                 % control horizon
nlobj.Ts = Ts;
nlobj.PredictionHorizon = p;       
nlobj.ControlHorizon = p;       

params = {Ts, B, K1, K2, x_ref, p};
nlobj.Model.IsContinuousTime = false;
nlobj.Model.NumberOfParameters = length(params);

nloptions = nlmpcmoveopt;
nloptions.Parameters = params;

mpc.Optimization.SolverOptions.Algorithm ='interior-point'; 

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
validateFcns(nlobj,x0,u0,[],params);
