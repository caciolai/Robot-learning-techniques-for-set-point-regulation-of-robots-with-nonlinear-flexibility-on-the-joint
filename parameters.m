%% COLLECT HERE ALL PARAMETERS ACROSS THE PROJECT
clear all;
close all;
clc;

%% Constants
nLinks = 2; % 2R

% towards negative y values so that it affects planar robot
g0 = [0 -9.80665 0]'; 

%% Rigid robot dynamic parameters and DH table
m = ones([nLinks 1]); % masses
l = ones([nLinks 1]); % link lengths
d = 0.4*ones([nLinks 1]); % distance (>0) of the center of mass from O_RF_i
I = zeros(3,3,nLinks); % the n inertia matrices
for k=1:nLinks
    I(:,:,k) = eye(3);
end


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

%% Time parameters
T = 1;      % Final time instant
Ts = 1e-2;   % Integration step    

%% Data generation parameters

% params for data generation
dgT = 1;
dgTs = 1e-3;

nTrajectories = 100;
reductionStep = 1;

%% MPC parameters

controlHorizon = 20;
% controlHorizon = 100;
lastSteps = 5;
maxIterations = 400;

% Weight matrices for LQR
Q = eye(8);           % to be tuned
Q(1:4, 1:4) = diag([10,10, 1,1]);
Q(5:8, 5:8) = zeros(4,4);
R = eye(2);           % to be tuned 
N = zeros(8,2);

params.T = T;
params.dgT = dgT;
params.Ts = Ts;
params.dgTs = dgTs;
params.B = B;
params.K1 = K1;
params.K2 = K2;
params.D = D;
params.p = controlHorizon;
params.maxIterations = maxIterations;
params.lastSteps = lastSteps;

%% Simulation configuration

% initial configuration
q0 = [0; 0];
theta0 = q0;
q0_dot = [0; 0];
theta0_dot = q0_dot;

% initial configuration
x0 = [q0; theta0; q0_dot; theta0_dot];
u0 = [0; 0];

% desired link position
q_ref = [pi/4 pi/4]';

params.x0 = x0;
params.u0 = u0;

clearvars all -except params