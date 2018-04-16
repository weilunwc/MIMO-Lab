clear;
close all;
clc;
load('Variables.mat');

%% Inverse based control Loop Shaping (in process....verify with professor)
fprintf('Inverse Based Controller Loop Shaping\n');
fprintf('=================================================\n');

% Tune crossover frequency to see good repsonse on control usage 
wc = 5;

% Performance weight
W_p=makeweight(100,wc,1/3);
Wp=eye(2)*W_p;

% Controller Weight
Wu=inv([25,0;0,25]);



% Loop shape = wc/s;
Ld = wc/s;

% To make semi-proper divide by large pole polynomial
% K_inv=(wc/s)*(inv(Gnom))*(1/(s+100));
%K_inv=minreal((wc/s)*(inv(Gnom)));

% Or just use loopsyn
[K_inv,CL_inv,GAM_inv]=loopsyn(Gunc,Ld);


% Get the final loop shape 
L_inv=Gunc*K_inv;
% Sensitivity function
S_inv=eye(2)-feedback(Gunc*K_inv,eye(2));

% Check RS/RP.
robstabmag_inv=robuststab(S_inv);
mu_stab_inv=1/robstabmag_inv.LowerBound;
fprintf('Mu for RS for Inverse based controller is %f\n',mu_stab_inv);
robperfmag_inv=robustperf(S_inv);
mu_perf_inv=1/robperfmag_inv.LowerBound;
fprintf('Mu for RP for Inverse based controller is %f\n',mu_perf_inv);

% Simulation
K = K_inv;
sim('MLC_Aero_model')
figure;
plot(simout.time,simout.signals.values);
title('Output using loopsyn controller');
figure;
plot(simcontrol.time,simcontrol.signals.values);
title('Control usage using loopsyn controller');





