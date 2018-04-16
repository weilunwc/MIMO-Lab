clear;
close all;
clc;
%% MLC Lab
%% Quanser Aero Helicopter control
quanser_aero_parameters;
quanser_aero_state_space;

s = tf('s');
Gunc = ss(A,B,C,D);
Gnom = tf(Gunc);

%% MIMO Poles and zeroes
z = tzero(minreal(Gnom));
p = eig(minreal(Gnom));
fprintf('Poles of the nominal system are:\n');
disp(p);
fprintf('Zeros of the nominal system are:\n');
disp(z);
fprintf('None\n');
fprintf('===================================================\n');

%% Bode Plot
figure;
bode(Gnom);
title('Nominal Plant')

figure;
bodemag(Gunc);
title('Uncertain Plant')

%% PM, GM
figure;
subplot(2,2,1)
margin(Gnom(1,1));
subplot(2,2,2)
margin(Gnom(1,2));
subplot(2,2,3)
margin(Gnom(2,1));
subplot(2,2,4)
margin(Gnom(2,2));


% Multiplicative uncertainity Weight
[Gunc_mult,info] = ucover(usample(Gunc,100),Gnom,[2 2],'InputMult');
WI = info.W1;
figure;
bodemag(((Gunc_mult-Gnom)/Gnom),WI);
title('Output Multiplicative Uncertainity Weight');
W_I_Pitch = WI(1,1);
W_I_Yaw = WI(2,2);

% Uncertainity Input
Delta_1=ultidyn('D1',[1,1]);
Delta_2=ultidyn('D2',[1,1]);
Del_I=[Delta_1,0;0,Delta_2];

% Save the important variables
save('Variables.mat', 'WI', 'W_I_Pitch', 'W_I_Yaw', 'Delta_1', 'Delta_2', 'Del_I', 'Gnom', 'Gunc', 'Gunc_mult', 's');






























