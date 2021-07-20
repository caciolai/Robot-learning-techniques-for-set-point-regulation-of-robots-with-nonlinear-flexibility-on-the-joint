%% MODEL OFFLINE TESTING (test effectiveness of learning model)
clc
clear all
close all forced hidden

addpath(genpath('../'));
addpath(genpath('./dataGeneration'));
addpath(genpath('./modelFunctions'));
addpath(genpath('./modelsTraining'));
addpath(genpath('./mpcFunctions'));
addpath(genpath('./savedData'));
addpath(genpath('./utils'));

parameters;

%% Load model
load('savedData/gpMdl.mat');
load('savedData/nnMdl.mat');

%% Evaluate the predictions
% test_pred_1 = predict(gpMdl{1}, test_data);
% test_pred_2 = predict(gpMdl{2}, test_data);

% Test on "exciting" trajectories
% u_max = params.maxTorque;
u_max = 1;
DeltaT = 100;
Ts = params.Ts;
T = params.T;
nSamples = T/Ts;

x0 = zeros(8,1);
u0 = [u_max;u_max];

xHistory = zeros(nSamples,8);
uHistory = zeros(nSamples,2);
psiReal = zeros(nSamples,2);
psiPredicted = zeros(nSamples,2); 

xk = x0;
u = u0;
for ct=1:nSamples
    if mod(ct, DeltaT) == 0
        u = -1*u;
    end
    
    xk = stateFunctionDT(xk, u, params);
    
    xHistory(ct,:) = xk';
    uHistory(ct,:) = u';
    psiReal(ct,:) = nonlinearElasticity(xk(1:2)-xk(3:4), params);
    psiPredicted(ct,:) = gpPredict(xk, gpMdl);
%     psiPredicted(ct,:) = nnMdl(xk(1:2)-xk(3:4));
end

y = psiReal;
y_hat = psiPredicted;
RMSE = sqrt(mean((y - y_hat).^2))  % Root Mean Squared Error

%% Show results

t = linspace(0,T,T/Ts)';
figure
hold on 
grid on

plot(t, abs(psiReal(:,1)-psiPredicted(:,1)));
plot(t, abs(psiReal(:,2)-psiPredicted(:,2)));

plot(t, repmat(RMSE(1), length(t),1));
plot(t, repmat(RMSE(2), length(t),1));

xlabel('[s]');
ylabel('[Nm]');

legend('Absolute error (Joint 1)', 'RMSE (Joint 1)', ...
    'Absolute error (Joint 2)', 'RMSE (Joint 2)', ...
    'Location', 'northwest');
title('Prediction error (GP)');
set(findall(gcf,'type','line'),'linewidth',2);
