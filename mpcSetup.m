%% MPC SETUP
addpath(genpath('./modelFunctions'));
addpath(genpath('./mpcFunctions'));
parameters;

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
x_ref = [q_ref; theta_ref; zeros(4, 1)]'; % must be row vector

%% MPC parameters
% nlmpcObj.Optimization.SolverOptions.Algorithm ='interior-point'; 
nlmpcObj.Optimization.SolverOptions.MaxIterations = mpcParams.maxIterations;

nlmpcObj.Model.IsContinuousTime = false;
nlmpcObj.Ts = Ts;
nlmpcObj.PredictionHorizon = controlHorizon;       
nlmpcObj.ControlHorizon = controlHorizon;  
nlmpcObj.Model.NumberOfParameters = 0;

mpcParams.x_ref = x_ref; 
simParams.x_ref = x_ref;

% System linearization and LQR
S = computeLQR(mpcParams, Q, R, N);

% Load GP model trained offline
% load('gpMdl.mat');
load('nnMdl.mat');

mpcParams.S = S;
% mpcParams.model = gpMdl;
mpcParams.model = nnMdl;

%% MPC model
nlmpcObj.Model.StateFcn = ...
    @(x, u) mpcStateFunctionDT(x, u, mpcParams); 
nlmpcObj.Model.OutputFcn = ...
    @(x, u) mpcOutputFunction(x, u, mpcParams);

%% MPC cost
nlmpcObj.Optimization.CustomCostFcn = ...
    @(x,u,e,data) mpcCostFunction(x,u,e,data,mpcParams);

nlmpcObj.Optimization.ReplaceStandardCost = true;

u_max = 1000;
nlmpcObj.ManipulatedVariables(1).Min = -u_max;
nlmpcObj.ManipulatedVariables(1).Max = +u_max;
nlmpcObj.ManipulatedVariables(2).Min = -u_max;
nlmpcObj.ManipulatedVariables(2).Max = +u_max;

%% MPC constraints
nlmpcObj.Optimization.CustomIneqConFcn = ...
    @(x,u,e,data) mpcInequalityConstraints(x,u,e,data,mpcParams);


%% Validate model
validateFcns(nlmpcObj,x0,u0);

%% Cleanup
clearvars -except nlmpcObj x0 u0 mpcParams simParams
