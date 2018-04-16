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

%% Mu-Synthesis
fprintf('Mu Synthesis Controller\n');
fprintf('=================================================\n');

% Define Plant using sysic
% Build Generalized plant using sysic
systemnames = 'Wu Wp Gunc';
inputvar = '[w{2};u{2}]';
outputvar = '[Wu;Wp;-w-Gunc]';
input_to_Gunc = '[u]';
input_to_Wp = '[w+Gunc]';
input_to_Wu = '[u]';
cleanupsysic = 'yes';  
PMU = sysic;

[K_MU, CL_MU,GAM_MU]=dksyn(PMU,2,2);

S_MU=eye(2)-feedback(Gunc*K_MU,eye(2));
N_MU=lft(PMU,K_MU);
% Check RS/RP
STABMARG= robuststab(N_MU);
mu_stabMU = 1/STABMARG.LowerBound;
fprintf('The mu for RS using dksyn is: %f\n',mu_stabMU);
% Use robustperf to compute mu
STABMARG= robustperf(N_MU);
mu_perfMU = 1/STABMARG.LowerBound;
fprintf('The mu for RP using dksyn is: %f\n',mu_perfMU);

% Simulation
K=K_MU;
sim('MLC_Aero_model')
figure;
plot(simout.time,simout.signals.values);
title('Output using dksyn controller');
figure;
plot(simcontrol.time,simcontrol.signals.values);
title('Control usage using dksyn controller');



% Save example controller
if	exist('Controller.mat', 'file')
	save('Controller.mat', 'K_MU', '-append');
else
	save('Controller.mat', 'K_MU');
end
