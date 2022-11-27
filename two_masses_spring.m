clear all; close all; clc;
%% given values and variables
M1 = 6; 
M2 = 2;
B1 = 6;
B2 = 0.4;
K = 11;
L = 8;
% Matrix A,B,C,D after linearzation
A = [0 1 0 0;...
    -K/M1 -B1/M1 K/M1 0;...
    0 0 0 1;...
    K/M2 0 -K/M2 -B2/M2];
B = [0;0;0;0];
C = [1 0 0 0];
D = L;
sys = ss(A,B,C,D);
s = tf('s');
[a,b] = ss2tf(A,B,C,D);
%%part 2 - observablitiy and controllability check
Ob = obsv (A,C);
rank_ob = rank(Ob);
%% part 3 - intial conditions responses
x0_1 = [1 5 -1 5];
x0_2 = [1 0 -1 0];

figure();
subplot(1,3,1);
initial (sys,x0_1);
title('response with intial condition [1;5;-1;5]');
subplot(1,3,2);
initial(sys,x0_2);
title('response with intial condition [1;0;-1;0]');
 
%% slow observer (X2)
%poles of system
subplot(1,3,3);
poles = pole(sys);
%plot pole zero domain and determine which pole is dominant
pzplot(tf([a],[b]));
grid on
desired_pole = real(max(poles))*2
desired_poles = [desired_pole desired_pole...
    desired_pole desired_pole];
%calculate L 
L_x2 = acker(A', C',desired_poles);
AL = A - L_x2'.*C
sys_speed_x2 = ss(AL, L_x2', C, D);
B_new_x2 = sys_speed_x2.B;
C_new_x2 = sys_speed_x2.C;
D_new_x2 = sys_speed_x2.D;

%% fast observer (X10)
desired_pole_fast = real(max(poles))*10;
desired_poles_fast = [desired_pole_fast desired_pole_fast...
    desired_pole_fast desired_pole_fast];
%calculate L 
L_fast = acker(A', C',desired_poles_fast);
AL_fast = A - L_fast'.*C
sys_speed_x10 = ss(AL_fast, L_fast', C, D);
B_new_x10 = sys_speed_x10.B;
C_new_x10 = sys_speed_x10.C;
D_new_x10 = sys_speed_x10.D;

% x2 speed with/out noise
N = 1000;
t = linspace(1,100,N);
%no noise
[y_observed, t, x_actual] = initial(sys, x0_2, t);
[y_x2, t, x_x2] = lsim(sys_speed_x2, y_observed, t-t(1));

%with noise
noisevec = sqrt(2)*randn(N,1);
y_with_noise =  y_observed + noisevec;  

[y_noise_x2, ~, x_x2_noise] = lsim(sys_speed_x2, y_with_noise,t);

figure();
plot(t,x_actual(:,1),t,x_x2(:,1),t,x_x2_noise(:,1));
title('x_2 With/out Noise for x2 faster (for initial condition x0_1)');
xlabel('time[sec]');
ylabel('x_2 [m]');
legend('actual x2','estimated x2 without noise','estimated x2 with noise');
% x10 faster observer
[y_x10, t, x_x10] = lsim(sys_speed_x10, y_observed, t-t(1));

[y_x10_noise, ~, x_x10_noise] = lsim(sys_speed_x10, y_with_noise,t);

figure();
plot(t,x_actual(:,1), t,x_x10(:,1),t,x_x10_noise(:,1));
title('x_2 With/out Noise for x10 faster (for initial condition x0_1)');
xlabel('time[sec]');
ylabel('x_2 [m]');
legend('actual x2','estimated x2 without noise','estimated x2 with noise');
