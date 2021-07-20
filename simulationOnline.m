%% MPC SIMULATION WITH ONLINE LEARNING

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
% robotModel;
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


dataset = params.dataset;
params.model = gpTrain(dataset);
training = true;
theta_dot_old = [0;0];

%% Simulate closed-loop system (simulazione vera: solo il primo punto)
retrain_interval = params.retrainInterval;
collect_interval = params.collectInterval;

for ct = 1:nSamples
    tau_g = g(q_ref);
%     fprintf("t = %.4f\n", ct * Ts);
%     state = xk(1:4)'
    
    [mv,nloptions,info] = nlmpcmove(nlmpcObj,xk,mv);
    tau = tau_g + mv; % gravity compensation
    
    xk = stateFunctionDT(xk, tau, params);
    
    % Reconstruct elasticity
    if size(dataset,1) < params.datasetDimension
        if mod(ct, collect_interval) == 0
            q = xk(1:2);
            theta = xk(3:4);
            theta_dot = xk_dot(3:4);
            theta_ddot = xk_dot(7:8);
            psi = B*theta_ddot + D*theta_dot - tau;

            dataset(end+1, :) = [q-theta; psi]';
        end
    
        if mod(ct, retrain_interval) == 0
            params.model = gpTrain(dataset); 
        end
    end
    
    xHistory(ct,:) = xk';
    uHistory(ct,:) = mv';
end

%% Plot closed-loop response
plotResults;

