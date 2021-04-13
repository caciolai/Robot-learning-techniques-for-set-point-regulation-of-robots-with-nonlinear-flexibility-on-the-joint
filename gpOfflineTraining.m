%% GP MODEL OFFLINE TRAINING
close all;
clear all;
clc;

%% Load data
disp("Loading data...");
load('data.mat');
dataset = [input; output];

% rng('default') % For reproducibility
cols = size(dataset,2);
disp("Done.");

%% Prepare dataset for training
% % Shuffle data columns (Meglio farlo solo sul training set?)
% index = randperm(cols);
% dataset = dataset(:,index);

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

%% Save model

gpMdl = {gpMdl_1;gpMdl_2};
save('gpMdl.mat', 'gpMdl');
disp("Model saved.");