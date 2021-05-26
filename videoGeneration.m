%% Robot initialization
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

parameters;
Rob = getRobot();

%% Trajectory data

nLinks = 2;
reductionStep = params.reductionStep;
dgT = params.dgT;
dgTs = params.dgTs;
step = dgTs * reductionStep;

q0      = [0;0];
q0_dot  = [0;0];
q0_ddot = [0;0];

qf      = [pi/4;pi/4];
qf_dot  = [0;0];
qf_ddot = [0;0];

[qd, ~, ~] = get_Trajectory_Desired(q0, q0_dot, q0_ddot, qf, qf_dot, qf_ddot,dgT,dgTs,nLinks);

%% Animation
%Initialize video
Video = VideoWriter('savedData\Animation');   % Open video file
Video.FrameRate = 10;               % Can adjust this
open(Video)

figure('Name', 'Robot Animation')
tiledlayout(1,1);

Rob.plot([qd(1,1) qd(2,1)],'trail','-.r')
 
for i = 1: 10 : size(qd, 2)
   Rob.animate([qd(1,i) qd(2,i)])   % Move the robot along qd
   pause(0.01)                      % Pause and grab frame
   frame = getframe(gcf);           % Get frame
   writeVideo(Video, frame);        % Add frame to video
end

close(Video)

