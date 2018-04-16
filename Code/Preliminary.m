clear;
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
fprintf('None\n');
fprintf('===================================================\n');
%% Bode Plot
figure;
bode(Gnom);
figure;
bodemag(Gunc);

%% PM, GM
figure;
[PM11,GM11]=margin(Gnom(1,1));
figure;
[PM12,GM12]=margin(Gnom(1,2));
figure;
[PM21,GM21]=margin(Gnom(2,1));
figure;
[PM22,GM22]=margin(Gnom(2,2));
%% Simulink model

%% Weights/ Uncertainity

% Multiplicative uncertainity Weight
[Gunc_mult,info] = ucover(usample(Gunc,100),Gnom,[2 2],'InputMult');
WI=info.W1;
figure;
bodemag(((Gunc_mult-Gnom)/Gnom),WI);
title('output Multiplicative Uncertainity Weight');
W_I_Pitch=WI(1,1);


