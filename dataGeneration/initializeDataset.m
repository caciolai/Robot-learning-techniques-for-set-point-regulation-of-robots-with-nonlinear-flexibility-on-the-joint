%% 
addpath(genpath('./modelFunctions'));
addpath(genpath('./modelsTraining'));
addpath(genpath('./mpcFunctions'));
addpath(genpath('./savedData'));
addpath(genpath('./utils'));

%%
K1 = params.K1;
K2 = params.K2;

numPoints = params.datasetDimensionInit;
points = 2*pi*rand(numPoints, 2) - pi;

%%
dataset = zeros(numPoints, 4);
for i=1:numPoints
    phi = points(i,:)';
    psi = nonlinearElasticity(phi, params);
    dataset(i,:) = [phi; psi]';
end

params.dataset = dataset;