
%% Offline training with NN
%https://it.mathworks.com/help/deeplearning/ug/workflow-for-neural-network-design.html
clear all
close all

%% Load data
disp("Loading data...");
load('data.mat');
dataset = [input;output];
cols = size(dataset,2);
disp("Done.");

%% Prepare dataset for training
% Shuffle data columns (Meglio farlo solo sul training set?)
% index = randperm(cols);
% dataset = dataset(:,index);

disp("Preparing dataset...");
% 80% training 20% test
train_size = round(0.8*cols); % test_size = cols - train_size;

train_data = input(:,1:train_size);
train_target = output(:,1:train_size); % data to train the model

test_data = input(:,train_size+1:end);
test_target = output(:,train_size+1:end); % data to test the model

%create the network object with 25 neurons in the hidden layer
% net = fitnet(25);
% view(net);
num_neurons = 16;
layers = [
    sequenceInputLayer(4,'Name','Input')               % Input Layer
    fullyConnectedLayer(num_neurons,'Name','FC_1') 
    fullyConnectedLayer(num_neurons,'Name','FC_2') 
    fullyConnectedLayer(2,'Name','Output')
    
    regressionLayer('Name','Regression')];                            % Regression Layer
% Computes the half-mean-squared-error loss for regression problems. 
    % For typical regression problems, a regression layer must follow the 
    % final fully connected layer.
    
% Visualization of the NN
layers
lgraph = layerGraph(layers);
figure
plot(lgraph)


% %train the neural network
% %samples are automatically divided into training, validation and test sets
% [net,tr] = train(net,input,output);
%% TRAIN THE NETWORK

% Specify training options
maxEpochs = 20;
miniBatchSize = 16;
% maxEpochs |  RMSE
%   10          21.3-18.9-23-21.8
%   15          19.4-19.15-20.1-20.4
%   20          17.8-18.5-17.6-17.65-19.6-17.55
%   25          18.8-20.3-17.5-19.1-20.2
%   35          25-17.15-20.6-23.4-19.6          


options = trainingOptions('adam', ...   % Adaptive Moment Estimation % rmsprop 18.77-25-25-24-19.4 adam 20.2-20-19.8-19.9-18.6
    'MaxEpochs',maxEpochs, ...          % Maximum Number of Epochs
    'MiniBatchSize',miniBatchSize, ...  % Size of Mini Batches
    'InitialLearnRate',0.0095, ...        % Initial learning rate 0.005x 0.0075x 0.009x 0.0095v 
    'GradientThreshold',1, ...          % Define the threshold to start clipping gradient (to prevent gradient explosion)
    'Shuffle','never', ...              % Do not shuffle data
    'Plots','training-progress',...     % Options for plotting
    'Verbose',0);

% Actual Training
net = trainNetwork(train_data,train_target,layers,options);


%check performance (measured in terms of squared error)
% nntraintool
% nntraintool('close')
% plotperform(tr)

% %testing the NN
% test_input = input(:,tr.testInd);
% test_output = output(:,tr.testInd);

% testY = net(test_data);
% Make predictions (actual testing)

test_pred = double(predict(net,test_data,'MiniBatchSize',1));   % No padding

% perf = mse(net,test_target,double(YPred)) %returns the mean squared error

figure
title("Elastic term (first joint)");
hold on
grid on
plot(test_target(1,:));
plot(test_pred(1,:));
legend('true', 'predicted');
set(findall(gcf,'type','line'),'linewidth',1); 

figure
title("Elastic term (second joint)");
hold on
grid on
plot(test_target(2,:));
plot(test_pred(2,:));
legend('true', 'predicted');
set(findall(gcf,'type','line'),'linewidth',1); 




