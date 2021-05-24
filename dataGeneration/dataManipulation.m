%% DATA MANIPULATION
clear all
close all
clc

addpath(genpath('../'));
addpath(genpath('./dataGeneration'));
addpath(genpath('./modelFunctions'));
addpath(genpath('./modelsTraining'));
addpath(genpath('./mpcFunctions'));
addpath(genpath('./savedData'));
addpath(genpath('./utils'));

%% Load data
disp("Loading data...");
load('data_traj.mat');
data_traj_full = [input; output];

% Reducing data size
reduction_data_size = 10;

k = 1;
for i = 1 : reduction_data_size : size(data_traj_full,2)
    data_traj(:,k) = data_traj_full(:,i);
    k = k +1;
end

load('data_mpc.mat');
data_mpc_full = [input; output];

for i = 1 : size(data_mpc_full, 2)
    row(i) = data_mpc_full(3, i)^2 + data_mpc_full(4, i)^2;
end

[row, I] = sort(row); 

% Sorted dataset
data_mpc_full = data_mpc_full(:, I);

% Number of sampled points
n = 100;

% Probability to be selected, according to the norm of the elastic term
P = row./sum(row);

% Sampling
I_sampled = datasample(I, n, 'Replace', false, 'Weights', P);

for i = 1 : n
    data_mpc(:, i) = data_mpc_full(:, I_sampled(i));
end  

% Check sorting order
figure

t = linspace(1, size(data_mpc, 2), size(data_mpc, 2));

hold on
grid on
plot(t,data_mpc(3, :).^2 + data_mpc(4, :).^2)
xlabel('Number of sample')
ylabel('$\psi_1$', 'Interpreter', 'latex')
title('Sorted elastic term')

%% Save the dataset
dataset = [data_traj, data_mpc];

save('..\savedData\dataset.mat', 'dataset');
disp("Model saved.");





