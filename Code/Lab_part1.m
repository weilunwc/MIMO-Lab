cclear;
close all;
clc;
%% MLC Lab
%% Quanser Aero Helicopter control
quanser_aero_parameters;
quanser_aero_state_space;

s=tf('s');
Gunc=ss(A,B,C,D);
Gnom=tf(Gunc);

%% MIMO Poles and zeroes
eps=1e-6;
z=tzero(minreal(Gnom));
p=eig(minreal(Gnom));
fprintf('Poles of the nominal system are:\n');
disp(p);
fprintf('Zeros of the nominal system are:\n');
disp(z);
fprintf('===================================================\n');
%% Bode Plot
figure;
bode(Gnom);
figure;
bodemag(Gunc);

%% Simulink model

%% Weights/ Uncertainity

% Multiplicative uncertainity Weight
[Gunc_mult,info] = ucover(usample(Gunc,100),Gnom,[2 2],'InputMult');
WI=info.W1;
figure;
bodemag(((Gunc_mult-Gnom)/Gnom),WI);
title('output Multiplicative Uncertainity Weight');

% Performance Weight
wc=5;
W_p=makeweight(100,wc,1/3);
Wp=eye(2)*W_p;

% Controller Weight
Wu=inv([25,0;0,25]);

% Uncertainity
D1=ultidyn('D1',[1,1]);
D2=ultidyn('D2',[1,1]);
Del_I=[D1,0;0,D2];

%% Inverse based control Loop Shaping
fprintf('Inverse Based Controller Loop Shaping\n');
fprintf('=================================================\n');
% Loop shape = wc/s;
Ld=wc/s;
% To make proper divide by large pole polynomial twice
% K_inv=(wc/s)*(inv(Gnom))*(1/(s+100)^2);
% K_inv=(wc/s)*(inv(Gnom));
% Or just use loopsyn
[K_inv,CL_inv,GAM_inv]=loopsyn(Gunc,Ld);
L_inv=Gunc*K_inv;
% Sensitivity function
S_inv=inv(eye(2)+L_inv);
T_inv=eye(2)-S_inv;

% Check RS/RP.
robstabmag_inv=robuststab(S_inv);
mu_stab_inv=1/robstabmag_inv.UpperBound;
fprintf('Mu for RS for Inverse based controller is %f\n',mu_stab_inv);
robperfmag_inv=robustperf(S_inv);
mu_perf_inv=1/robperfmag_inv.UpperBound;
fprintf('Mu for RP for Inverse based controller is %f\n',mu_perf_inv);

% Simulation
K=K_inv;
sim('MLC_Aero_model')
figure;
plot(simout.time,simout.signals.values);
title('Output using loopsyn controller');
figure;
plot(simcontrol.time,simcontrol.signals.values);
title('Control usage using loopsyn controller');

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
S_LQG=eye(2)/(eye(2)+Gunc*K_LQG);
T_LQG=eye(2)-S_LQG;

% Check RS/RP
robstabmag_LQG=robuststab(S_LQG);
mu_stab_LQG=1/robstabmag_LQG.UpperBound;
fprintf('Mu for RS for LQG controller is %f\n',mu_stab_LQG);
robperfmag_LQG=robustperf(S_LQG);
mu_perf_LQG=1/robperfmag_LQG.UpperBound;
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
%% H2Syn
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
S_2=eye(2)/(eye(2)+Gunc*K_2);
T_2=eye(2)-S_2;

N_2=lft(PP2,K_2);
% Check RS/RP
robstabmag_LQG=robuststab(N_2);
mu_stab_2=1/robstabmag_LQG.UpperBound;
fprintf('Mu for RS for H2syn controller is %f\n',mu_stab_2);
robperfmag_LQG=robustperf(N_2);
mu_perf_2=1/robperfmag_LQG.UpperBound;
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
S_INF=inv(eye(2)+Gunc*K_INF);
T_INF=eye(2)-S_INF;
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
%% Mu-Synthesis
fprintf('Mu Synthesis Controller Controller\n');
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
S_MU=inv(eye(2)+Gunc*K_MU);
T_MU=eye(2)-S_MU;

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
%% Bode magnitude plot of all sensitivity functions:
bodemag(S_inv,S_LQG,S_2,S_INF,S_MU);
title('All Sensitivity Function plots');
legend('Loopsyn','LQG','H2syn','H-infsyn','dksyn');

%% Hinf Loop shaping


%% Testing

sim('MLC_Aero_model');
%% End of Problem

