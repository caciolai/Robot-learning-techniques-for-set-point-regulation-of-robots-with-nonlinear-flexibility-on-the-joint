%% 
addpath(genpath('./modelFunctions'));
addpath(genpath('./modelsTraining'));
addpath(genpath('./mpcFunctions'));
addpath(genpath('./savedData'));
addpath(genpath('./utils'));

K1 = params.K1;
K2 = params.K2;

numPoints = params.datasetDimensionInit;

load('savedData/data_exc.mat');
data_exc = [input output]';

%% 
for i = 1 : size(data_exc, 2)
    row(i) = data_exc(3, i)^2 + data_exc(4, i)^2;
end

[row, I] = sort(row); 

% Sorted dataset
data_exc = data_exc(:, I);

% Number of sampled points
n = 100;

% Probability to be selected, according to the norm of the elastic term
P = row./sum(row);

% Sampling
I_sampled = datasample(I, n, 'Replace', false, 'Weights', P);

data_exc_sampled = zeros(4, numPoints);
for i = 1 : n
    data_exc_sampled(:, i) = data_exc(:, I_sampled(i));
end

params.dataset = data_exc_sampled;
