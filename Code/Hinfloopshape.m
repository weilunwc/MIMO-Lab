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

%% Hinf Loop shaping (in process....verify with professor)

fprintf('Hinf Loop Shaping  Controller\n');
fprintf('=================================================\n');

% Desired loop shape
%Ld=[wc*(s-0.00001)/(s*(s+0.00001)),0;0,wc*(s-0.00001)/(s*(s+0.00001))];

Ld=[wc/s,0;0,wc/s];
W1=Ld/(Gnom);
W2=eye(2);
[K_INFLOOP,CL_INFLOOP,GAM_INFLOOP]=ncfsyn(Gunc,W1,W2);

S_INFLOOP=1-feedback(Gunc*K_INFLOOP,eye(2));
T_INFLOOP=eye(2)-S_INFLOOP;


% Check RS/RP
STABMARG= robuststab(K_INFLOOP*S_INFLOOP);
mu_stabINFLOOP = 1/STABMARG.LowerBound;
fprintf('The mu for RS using ncfsyn is: %f\n',mu_stabINFLOOP);
% Use robustperf to compute mu
STABMARG= robustperf(K_INFLOOP*S_INFLOOP);
mu_perfINFLOOP = 1/STABMARG.LowerBound;
fprintf('The mu for RP using ncfsyn is: %f\n',mu_perfINFLOOP);

% % Simulation does not work yet, make system semi-proper
% K=K_INFLOOP;
% sim('MLC_Aero_model')
% figure;
% plot(simout.time,simout.signals.values);
% title('Output using ncfsyn controller');
% figure;
% plot(simcontrol.time,simcontrol.signals.values);
% title('Control usage using ncfsyn controller');
%