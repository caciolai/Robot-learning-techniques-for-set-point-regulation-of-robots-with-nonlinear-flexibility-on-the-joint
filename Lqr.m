% Initialization
clear all
close all
clc

addpath(genpath('./modelFunctions'));

%% Parameters
% Motor inertia matrix
B = eye(2);

% Stiffness matrices (for elasticity)
K1 = eye(2) * 1000;
K2 = eye(2) * 10;

% Damping
D = eye(2) * 10; 

%% Reference
% desired link position
q_ref = [pi 0]';

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

%% Initialize symbolic state vector
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
fx = [q_dot; theta_dot; M(q)\(-psi - c(q, q_dot) - g(q) -D*q_dot); B\(psi-D*theta_dot)];
gx = [zeros(6,2); inv(B)];

%% Linearization
q_ref = [pi; 0]; % Desired position
x_bar = x_ref';  % Equilibrium point
% u_bar = 0

A = double(subs(jacobian(fx,x), x, x_bar));
B = double(subs(gx, x, x_bar));

%% LQR
% Weight matrices
Q = [eye(8)];           % to be tuned
R = [eye(2)];           % to be tuned 
N = zeros(8,2);

[K,S,e] = lqr(A,B,Q,R,N);

% Check stability
e % Closed loop eigenvalues (A-B*K)

