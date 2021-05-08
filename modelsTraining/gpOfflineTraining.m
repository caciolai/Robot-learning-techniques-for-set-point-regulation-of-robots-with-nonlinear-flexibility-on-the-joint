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

%% Load data
disp("Loading data...");
load('data_traj.mat');
data_traj = [input; output];

load('data_mpc.mat');
data_mpc_full = [input; output];

% Reducing data size
reduction_data_size = 10;
k = 1;
for i = 1 : reduction_data_size : size(data_mpc_full,2)
    data_mpc(:,k) = data_mpc_full(:,i);
    k = k +1;
end

dataset = [data_traj, data_mpc];

rng('default') % For reproducibility
cols = size(dataset,2);
disp("Done.");

%% Prepare dataset for training
% Shuffle data columns 
index = randperm(cols);
dataset = dataset(:,index);

disp("Preparing dataset...");
% 80% training 20% test
train_size = round(0.8*cols); % test_size = cols - train_size;

train_data = input(:,1:train_size)';
train_target_1 = output(1,1:train_size)'; % Joint 1 data to train the model
train_target_2 = output(2,1:train_size)'; % Joint 2 data to train the model

test_data = input(:,train_size+1:end)';
test_target_1 = output(1,train_size+1:end)'; % Joint 1 data to test the model
test_target_2 = output(2,train_size+1:end)'; % Joint 2 data to test the model
disp("Done.");

%% Train the GP
% https://au.mathworks.com/help/stats/subset-of-data-approximation-for-gpr-models.html
% https://au.mathworks.com/help/stats/fitrgp.html#d122e337624
% Di default lui mette 'sd' quando il numero di punti di training è > 2000.
% Possiamo anche scegliere noi la dimensione del subset (active set) con 'ActiveSetSize',dim

disp("Training first GP model...");
tic
gpMdl_1 = fitrgp(train_data, train_target_1,'FitMethod','sd','ActiveSetMethod','entropy', 'PredictMethod','sd', 'Standardize', true);
disp("Done.");
toc

disp("Training second GP model...");
tic
gpMdl_2 = fitrgp(train_data, train_target_2,'FitMethod','sd','ActiveSetMethod','entropy', 'PredictMethod','sd', 'Standardize', true);
disp("Done.");
toc

%% Evaluate the predictions
[test_pred_1,~,~] = predict(gpMdl_1, test_data);
[test_pred_2,~,~] = predict(gpMdl_2, test_data);

figure
title("Elastic term (first joint)");
hold on
grid on
plot(test_target_1);
plot(test_pred_1);
legend('true', 'predicted');
set(findall(gcf,'type','line'),'linewidth',1); 

figure
title("Elastic term (second joint)");
hold on
grid on
plot(test_target_2);
plot(test_pred_2);
legend('true', 'predicted');
set(findall(gcf,'type','line'),'linewidth',1); 

figure
subplot(2,1,1)
title("Elastic term error(first joint)");
hold on
grid on
plot(test_target_1 - test_pred_1);
legend('Error link 1');
set(findall(gcf,'type','line'),'linewidth',1); 

subplot(2,1,2)
title("Elastic term error(second joint)");
hold on
grid on
plot(test_target_2 - test_pred_2);
legend('Error link 2');
set(findall(gcf,'type','line'),'linewidth',1); 

%% Save model

gpMdl = {compact(gpMdl_1);compact(gpMdl_2)};
save('..\savedData\gpMdl.mat', 'gpMdl');
disp("Model saved.");