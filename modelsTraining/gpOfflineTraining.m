%% GP MODEL OFFLINE TRAINING
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

load('dataset.mat')
% input = dataset(1:2,:);
% output = dataset(3:4,:);
input = dataset(1:2,1:10);
output = dataset(3:4,1:10);

% load('data_exc.mat');
% dataset = [input output]';

%% Prepare dataset for training

% rng('default') % For reproducibility
% cols = size(dataset,2);
% index = randperm(cols);
% dataset = dataset(:,index);

train_data = input';
train_target_1 = output(1,:)'; % Joint 1 data to train the model
train_target_2 = output(2,:)'; % Joint 2 data to train the model

%% Train the GP
% https://au.mathworks.com/help/stats/subset-of-data-approximation-for-gpr-models.html
% https://au.mathworks.com/help/stats/fitrgp.html#d122e337624
% Di default lui mette 'sd' quando il numero di punti di training è > 2000.
% Possiamo anche scegliere noi la dimensione del subset (active set) con 'ActiveSetSize',dim

disp("Training...");
tic

gpMdl = gpTrain(dataset);

disp("Done.");
toc

%% Save model

save('savedData/gpMdl.mat', 'gpMdl');
disp("Model saved.");