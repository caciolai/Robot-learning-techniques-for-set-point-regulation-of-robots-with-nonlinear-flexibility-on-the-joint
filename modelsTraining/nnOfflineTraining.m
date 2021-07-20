%% Offline training with NN
%https://it.mathworks.com/help/deeplearning/ug/workflow-for-neural-network-design.html
clear all
close all

addpath(genpath('../'));
addpath(genpath('./dataGeneration'));
addpath(genpath('./modelFunctions'));
addpath(genpath('./modelsTraining'));
addpath(genpath('./mpcFunctions'));
addpath(genpath('./savedData'));
addpath(genpath('./utils'));

parameters;

%% Load data

% load('data_traj.mat');
% data_traj = [input; output];
% 
% load('data_mpc.mat');
% data_mpc_full = [input; output];

% load('dataset.mat')
% input = dataset(1:2,:);
% output = dataset(3:4,:);

load('data_exc.mat');

X = input';
T = output';

%% Create network
setdemorandstream(491218382);
net = fitnet(32);
net = configure(net,X,T);
view(net)

%% Train the model
[net,tr] = train(net,X,T,'useGPU','yes');
nntraintool
nntraintool('close')
plotperform(tr)


%% Save model
nnMdl = net;
save('savedData/nnMdl.mat', 'nnMdl');