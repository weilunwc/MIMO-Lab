clear;
close all;
clc;
load('Variables.mat');

% Tune crossover frequency to see good repsonse on control usage 
wc = 5;

% Performance weight
W_p=makeweight(100,wc,1/3);
Wp=eye(2)*W_p;

% Controller Weight
Wu=inv([25,0;0,25]);


%% LQG
fprintf('LQG Controller\n');
fprintf('=================================================\n');
% Divide performance weight by large pole ot make it proper. LQG does not
% like improper weight.
W_p_LQG=Wp*(1/(s+100));

% Noise Varaince from Manual
QXU = blkdiag(diag([200,150,100,200]),50,50);
QWV = blkdiag(10*eye(4),1*eye(2));  
K_LQG = -lqg(Gunc.NominalValue,QXU,QWV);
% Sensitivity function
S_LQG=eye(2)-feedback(Gunc*K_LQG,eye(2));

% Check RS/RP
robstabmag_LQG=robuststab(S_LQG);
mu_stab_LQG=1/robstabmag_LQG.LowerBound;
fprintf('Mu for RS for LQG controller is %f\n',mu_stab_LQG);
robperfmag_LQG=robustperf(S_LQG);
mu_perf_LQG=1/robperfmag_LQG.LowerBound;
fprintf('Mu for RP for LQG controller is %f\n',mu_perf_LQG);

% Simulation
K=K_LQG;
sim('MLC_Aero_model')
figure;
plot(simout.time,simout.signals.values);
title('Output using LQG controller');
figure;
plot(simcontrol.time,simcontrol.signals.values);
title('Control usage using LQG controller');