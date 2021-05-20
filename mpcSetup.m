%% MPC SETUP

addpath(genpath('./dataGeneration'));
addpath(genpath('./modelFunctions'));
addpath(genpath('./modelsTraining'));
addpath(genpath('./mpcFunctions'));
addpath(genpath('./savedData'));
addpath(genpath('./utils'));
parameters;


q_ref = params.q_ref;
K1 = params.K1;
K2 = params.K2;

%% Create nonlinear MPC object
nx = 8; % number of state variables (q, theta, q_dot, theta_dot) 
ny = 8; % number of output variables (full state)
nu = 2; % number of input variables (torques)

nlmpcObj = nlmpc(nx,ny,nu);

%% Plan trajectory


% SET POINT FOR NONLINEAR ELASTICITY
% vogliamo che all'equilibrio il termine elastico compensi la gravità
g_ref = g(q_ref);

prob = eqnproblem;
x = optimvar('x',2);
eqn1 = -K1(1,1) * (q_ref(1) - x(1)) - K2(1,1) * (q_ref(1) - x(1))^3 - g_ref(1) == 0;
eqn2 = -K1(2,2) * (q_ref(2) - x(2)) - K2(2,2) * (q_ref(2) - x(2))^3 - g_ref(2) == 0;
sol.x = [0.0, 0.0];

prob.Equations.eqn1 = eqn1;
prob.Equations.eqn2 = eqn2;

[sol,fval,exitflag] = solve(prob,sol);
theta_ref = sol.x;

% desired equilibrium (no velocity)
params.x_ref = [q_ref; theta_ref; zeros(4, 1)]'; % must be row vector

%% MPC parameters

nlmpcObj.Model.IsContinuousTime = false;
nlmpcObj.Ts = params.Ts;
nlmpcObj.PredictionHorizon = params.controlHorizon;  
nlmpcObj.ControlHorizon = params.controlHorizon;       
nlmpcObj.Model.NumberOfParameters = 0;

 
% System linearization and LQR
Q = params.Q;
R = params.R;
N = params.N;
params.S = computeLQR(params, Q, R, N);

%% MPC model
nlmpcObj.Model.StateFcn = ...
    @(x, u) mpcStateFunctionDT(x, u, params); 
nlmpcObj.Model.OutputFcn = ...
    @(x, u) mpcOutputFunction(x, u, params);

%% MPC cost
nlmpcObj.Optimization.CustomCostFcn = ...
    @(x,u,e,data) mpcCostFunction(x,u,e,data,params);

nlmpcObj.Optimization.ReplaceStandardCost = true;

u_max = params.maxTorque;
nlmpcObj.ManipulatedVariables(1).Min = -u_max;
nlmpcObj.ManipulatedVariables(1).Max = +u_max;
nlmpcObj.ManipulatedVariables(2).Min = -u_max;
nlmpcObj.ManipulatedVariables(2).Max = +u_max;

%% MPC constraints
nlmpcObj.Optimization.CustomIneqConFcn = ...
    @(x,u,e,data) mpcInequalityConstraints(x,u,e,data,params);


%% Validate model
x0 = params.x0;
u0 = params.u0;
validateFcns(nlmpcObj,x0,u0);

%% Initialize dataset
% initializeDataset;

%% Cleanup
clearvars -except nlmpcObj params
