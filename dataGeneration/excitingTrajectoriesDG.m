%% Generate exciting trajectories

addpath(genpath('../'));
addpath(genpath('./utils'));
addpath(genpath('./modelFunctions'));

parameters;

%%

nTrajectories = params.nTrajectories;
T = params.T;
Ts = params.Ts;
nSamples = T/Ts;

x0 = zeros(8,1);

xHistory = zeros(nSamples,8);
uHistory = zeros(nSamples,2);
psiReal = zeros(nSamples,2);
psiPredicted = zeros(nSamples,2); 

% increasing amplitude and frequency
u_max = 1e2;
amp = linspace(1, u_max, nTrajectories);
period = linspace(100, 2, nTrajectories);

input = zeros(nTrajectories*nSamples, 2);
output = zeros(nTrajectories*nSamples, 2);

bar = waitbar(0,'Data generation ...');

for i=1:nTrajectories
    
    waitbar(i/nTrajectories,bar);
    xk = x0;
    DeltaT = period(i);
    A = amp(i);
    u = [A;A];
    for ct=1:nSamples
        if mod(ct, DeltaT) == 0
            u = -1*u;
        end
        
        xk = stateFunctionDT(xk, u, params);
        q = xk(1:2);
        theta = xk(3:4);
        
        psi = nonlinearElasticity(q-theta, params);
        
        input((i-1)*nSamples+ct, :) = (q-theta)';
        output((i-1)*nSamples+ct, :) = psi';
   end
end
close(bar);

%% Subsample
nSamplesFull = nSamples*nTrajectories;
input_full = input;
output_full = output;

reductionStep = params.reductionStep;
nSamples = nSamplesFull / reductionStep;
input = zeros(nSamples,2);
output = zeros(nSamples,2);

k = 1;
for i=1:nSamplesFull
   if mod(i-1,reductionStep) == 0
     input(k,:) = input_full(i,:);
     output(k,:) = output_full(i,:);
     k = k+1;
   end
end

%% Cleanup
close all;
save('savedData\data_exc.mat','input','output');
clear all;