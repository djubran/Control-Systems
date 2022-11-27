clc; close all;
J = 0.01;
b = 0.1;
K = 0.01;
R = 1 ;
L = 0.5;
s = tf('s');
P_small_resistance= K/((J*s+b)*(L*s+R)+K^2);
linearSystemAnalyzer('step', P_small_resistance, 0:0.1:20);
%step(P_motor)
J1 = 0.01;
b1 = 0.1;
K1 = 0.01;
R1 = 5 ;
L1 = 0.5;
s = tf('s');
P_big_resistance = K1/((J1*s+b1)*(L1*s+R1)+K1^2);
