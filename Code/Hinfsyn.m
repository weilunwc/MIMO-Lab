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

%% H-inf
fprintf('H-inf Controller\n');
fprintf('=================================================\n');

% Build Generalized plant using sysic
systemnames = 'WI Wu Wp Gnom';
inputvar = '[ud{2};w{2};u{2}]';
outputvar = '[WI;Wu;Wp;-w-Gnom]';
input_to_Gnom = '[u+ud]';
input_to_WI = '[u]';
input_to_Wp = '[w+Gnom]';
input_to_Wu = '[u]';
cleanupsysic = 'yes';  
PP = sysic;
[K_INF, CL_INF,GAM_INF]=hinfsyn(PP,2,2);

% Sensitivity Function
S_INF=eye(2)-feedback(Gunc*K_INF,eye(2));

N_INF=lft(PP,K_INF);
% Check RS/RP
STABMARG= robuststab(N_INF);
mu_stabinf = 1/STABMARG.LowerBound;
fprintf('The mu for RS using hinfsyn is: %f\n',mu_stabinf);
STABMARG= robustperf(N_INF);
mu_perfinf = 1/STABMARG.LowerBound;
fprintf('The mu for RP using hinfsyn is: %f\n',mu_perfinf);

% Simulation
K=K_INF;
sim('MLC_Aero_model')
figure;
plot(simout.time,simout.signals.values);
title('Output using Hinfsyn controller');
figure;
plot(simcontrol.time,simcontrol.signals.values);
title('Control usage using hinfsyn controller');

