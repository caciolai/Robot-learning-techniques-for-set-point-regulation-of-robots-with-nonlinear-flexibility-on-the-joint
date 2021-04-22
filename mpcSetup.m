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

%% Simulation parameters

% desired link position
q_ref = [pi/4 pi/4]';

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

% simulation time
T = 10.0; % [s]

% initial configuration
q0 = [0; 0];
theta0 = q0;
q0_dot = [0; 0];
theta0_dot = q0_dot;

x0 = [q0; theta0; q0_dot; theta0_dot];
u0 = [0; 0];

%% System linearization and LQR
syms x [8 1] 'real'         % x1 = q1
                            % x2 = q2
                            % x3 = theta1
                            % x4 = theta2
q = [x1; x2];               % x5 = q1_dot
theta = [x3; x4];           % x6 = q2_dot
q_dot = [x5; x6];           % x7 = theta1_dot
theta_dot = [x7; x8];       % x8 = theta2_dot            

%     psi = linearElasticity(x, K1);
psi = nonlinearElasticity(x, K1, K2);

% Vector fields
fx = [q_dot; theta_dot; M(q)\(-psi - c(q, q_dot) - g(q) -D*(q_dot-theta_dot)); B\(psi-D*(theta_dot-q_dot))];
gx = [zeros(6,2); inv(B)];

x_bar = x_ref';  % Equilibrium point
% u_bar = 0

A_sys = double(subs(jacobian(fx,x), x, x_bar));
B_sys = double(subs(gx, x, x_bar));

% Weight matrices
Q = eye(8);           % to be tuned
Q(1:4, 1:4) = diag([10,10, 1,1]);
Q(5:8, 5:8) = zeros(4,4);
R = eye(2);           % to be tuned 
N = zeros(8,2);

% lqr for terminal cost
[~,S,~] = lqr(A_sys,B_sys,Q,R,N);

%% MPC parameters
Ts = 1e-2;                              % integration step
p = 100;                                 % control/prediction horizon
nlmpcObj.Ts = Ts;
nlmpcObj.PredictionHorizon = p;       
nlmpcObj.ControlHorizon = p;       

% li passo "sotto banco"
params.Ts = Ts;
params.B = B;
params.K1 = K1;
params.K2 = K2;
params.D = D;
params.x_ref = x_ref;
params.p = p;
params.S = S;

nlmpcObj.Model.IsContinuousTime = false;
nlmpcObj.Model.NumberOfParameters = 0;

% nlmpcObj.Optimization.SolverOptions.Algorithm ='interior-point'; 
nlmpcObj.Optimization.SolverOptions.MaxIterations = 400;

%% MPC model
nlmpcObj.Model.StateFcn = ...
    @(x, u) mpcStateFunctionDT(x, u, params); 
nlmpcObj.Model.OutputFcn = ...
    @(x, u) mpcOutputFunction(x, u, params);

%% MPC cost
nlmpcObj.Optimization.CustomCostFcn = ...
    @(x,u,e,data) mpcCostFunction(x,u,e,data,params);

nlmpcObj.Optimization.ReplaceStandardCost = true;

u_max = 1000;
nlmpcObj.ManipulatedVariables(1).Min = -u_max;
nlmpcObj.ManipulatedVariables(1).Max = +u_max;
nlmpcObj.ManipulatedVariables(2).Min = -u_max;
nlmpcObj.ManipulatedVariables(2).Max = +u_max;

%% MPC constraints
nlmpcObj.Optimization.CustomIneqConFcn = ...
    @(x,u,e,data) mpcInequalityConstraints(x,u,e,data,params);


%% Validate model
validateFcns(nlmpcObj,x0,u0);
