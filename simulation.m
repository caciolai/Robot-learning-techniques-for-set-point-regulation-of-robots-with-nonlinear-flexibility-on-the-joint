%% MPC SIMULATION
clear all;
close all forced hidden;
clc;

addpath(genpath('../'));
addpath(genpath('./dataGeneration'));
addpath(genpath('./modelFunctions'));
addpath(genpath('./modelsTraining'));
addpath(genpath('./mpcFunctions'));
addpath(genpath('./savedData'));
addpath(genpath('./utils'));

%% Load parameters, model and setup mpc
% robotModel; % derive needed model terms
mpcSetup;

x0 = params.x0;
u0 = params.u0;
T = params.T;
Ts = params.Ts;
x_ref = params.x_ref;

nx = nlmpcObj.Dimensions.NumberOfStates;
nu = nlmpcObj.Dimensions.NumberOfInputs;
ny = nlmpcObj.Dimensions.NumberOfOutputs;

p = params.controlHorizon;

B = params.B;
D = params.D;

%% Initialize simulation
mv = u0;
xk = x0;

q_ref = x_ref(1:2)';
theta_ref = x_ref(3:4)';

nSamples = T/Ts;
% so that it is a multiple of p
nSamples = nSamples + (p - mod(nSamples, p)); 

% to record history of simulation
xHistory = zeros(nSamples,nx);
uHistory = zeros(nSamples,nu);

nloptions = nlmpcmoveopt;


%% Simulate closed-loop system 
for ct = 1:nSamples
    tau_g = g(q_ref);
    fprintf("t = %.4f\n", ct * Ts);
    state = xk(1:4)' % print current state
    
    [mv,nloptions,info] = nlmpcmove(nlmpcObj,xk,mv);
    tau = tau_g + mv; % gravity compensation
    
    xk = stateFunctionDT(xk, tau, params);
    
    xHistory(ct,:) = xk';
    uHistory(ct,:) = mv';
end

%% Plot closed-loop response
plotResults;
