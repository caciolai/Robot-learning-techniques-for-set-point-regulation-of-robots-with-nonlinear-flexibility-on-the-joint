
%% Offline training with NN
%https://it.mathworks.com/help/deeplearning/ug/workflow-for-neural-network-design.html
clear all;
close all;
clc;

%% Load data
load('data.mat');
X = input;
T = output;
dataset = [X;T];

%% Create network
setdemorandstream(491218382);
net = fitnet(32);
net = configure(net,X,T);
view(net)

%% TRAIN THE NETWORK
[net,tr] = train(net,X,T,'useGPU','yes');
nntraintool
nntraintool('close')
plotperform(tr)

%% Validation
testX = X(:,tr.testInd);
testT = T(:,tr.testInd);

testY = net(testX);
perf = mse(net,testT,testY)

figure
title("Elastic term (first joint)");
hold on
grid on
plot(testT(1,:));
plot(testY(1,:));
legend('true', 'predicted');
set(findall(gcf,'type','line'),'linewidth',1); 

figure
title("Elastic term (second joint)");
hold on
grid on
plot(testT(2,:));
plot(testY(2,:));
legend('true', 'predicted');
set(findall(gcf,'type','line'),'linewidth',1); 

%% Save
nnMdl = net;
save('nnMdl.mat', 'nnMdl');