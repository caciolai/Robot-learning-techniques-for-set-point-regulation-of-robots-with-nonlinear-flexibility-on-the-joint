%% COLLECT HERE ALL PARAMETERS ACROSS THE PROJECT
clear all;
close all;
clc;

%% Constants
nLinks = 2; % 2R

% towards negative y values so that it affects planar robot
params.g0 = [0 -9.80665 0]'; 

%% Rigid robot dynamic parameters and DH table
params.m = ones([nLinks 1]); % masses
params.l = ones([nLinks 1]); % link lengths
params.d = 0.4*ones([nLinks 1]); % distance (>0) of the center of mass from O_RF_i
params.I = zeros(3,3,nLinks); % the n inertia matrices
for k=1:nLinks
    params.I(:,:,k) = eye(3);
end


%% Elastic parameters

% Motor inertia matrix
params.B = eye(2);

% Stiffness matrices (for elasticity)
params.K1 = eye(2) * 1e3;
params.K2 = eye(2) * 1e2;

% Damping
params.D = eye(2) * 10; 

%% Time parameters
params.T = 1;      % Final time instant
params.Ts = 1e-3;   % Integration step    

%% Data generation parameters
% params for data generation
params.dgT = 1;
params.dgTs = 1e-3;

params.nTrajectories = 100;
params.reductionStep = 10;

params.datasetDimension = 200;
params.collectInterval = 10;
params.retrainInterval = 10 * params.collectInterval;

%% MPC parameters
params.controlHorizon = 30;
% params.controlHorizon = 100;
params.lastSteps = 5;
params.maxTorque = 100;

% Weight matrices for LQR
Q = eye(8);           % to be tuned
Q(1:4, 1:4) = diag([10,10, 1,1]);
Q(5:8, 5:8) = zeros(4,4);
params.Q = Q;
params.R = eye(2);           % to be tuned 
params.N = zeros(8,2);

%% Simulation configuration
% initial configuration
q0 = [-pi/2; 0];
theta0 = q0;
q0_dot = [0; 0];
theta0_dot = q0_dot;

% initial configuration
params.x0 = [q0; theta0; q0_dot; theta0_dot];
params.u0 = [0; 0];

% desired link position
params.q_ref = [pi/2, 0]';
% params.q_ref(1) = input('Desired link 1 position : ');
% params.q_ref(2) = input('Desired link 2 position : ');

%% Learning model
% Load model trained offline
load('gpMdl.mat');
% load('nnMdl.mat');

params.model = gpMdl;
% params.model = nnMdl;

%% Load dataset
% load('dataset.mat');
load('data_exc.mat');
dataset = [input; output];

params.dataset = dataset;

%% Cleanup

clearvars all -except params