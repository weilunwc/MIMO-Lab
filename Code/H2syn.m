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

% H2Syn
fprintf('H2syn Controller\n');
fprintf('=================================================\n');
% Divide performance weight by large pole ot make it proper. H2syn does not
% like improper weight.
Wp_2=Wp*(1/(s+100));

% Build Generalized plant using sysic
systemnames = 'WI Wu Wp_2 Gnom';
inputvar = '[ud{2};w{2};u{2}]';
outputvar = '[WI;Wu;Wp_2;-w-Gnom]';
input_to_Gnom = '[u+ud]';
input_to_WI = '[u]';
input_to_Wp_2 = '[w+Gnom]';
input_to_Wu = '[u]';
cleanupsysic = 'yes';  
PP2 = sysic;

[K_2, CL_2,GAM_2]=h2syn(PP2,2,2);

% Sensitivity function
S_2=eye(2)-feedback(Gunc*K_2,eye(2));

N_2=lft(PP2,K_2);
% Check RS/RP
robstabmag_LQG=robuststab(N_2);
mu_stab_2=1/robstabmag_LQG.LowerBound;
fprintf('Mu for RS for H2syn controller is %f\n',mu_stab_2);
robperfmag_LQG=robustperf(N_2);
mu_perf_2=1/robperfmag_LQG.LowerBound;
fprintf('Mu for RP for H2syn controller is %f\n',mu_perf_2);

% Simulation
K=K_2;
sim('MLC_Aero_model')
figure;
plot(simout.time,simout.signals.values);
title('Output using H2syn controller');
figure;
plot(simcontrol.time,simcontrol.signals.values);
title('Control usage using H2syn controller');