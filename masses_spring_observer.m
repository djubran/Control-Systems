clear all;close all;clc
%transfer func.controllable form and its state space 
s = tf('s');
Gc = (s^2+s-2)/(s^3+3*s^2-4*s-12);
Gss = ss(Gc);

% desired  poles = -2,-2,-3
poles = [-2 -2 -3];
K1 = place(Gss.A, Gss.B,poles);
Acl = A-B*K1;
syscl = ss(Acl,B,C,D);
Pcl = pole(syscl)
%%
%calculate  K such that we get poles displacement right we'll use ackr
%func. for it's easy use
K = acker(Gss.A, Gss.B, poles);
% system with vector state feedback step response
A_new = Gss.A-Gss.B*K;
desired_sys = ss(A_new, Gss.B, Gss.C, 0);
figure();
subplot(1,2,1); step(desired_sys);
title('Step Response');
%improvement to Gain to reach 1;
gain = -6
output_improved_gain = desired_sys*-6
subplot(1,2,2); step(output_improved_gain);
title('Step Response With Gain = -6');
