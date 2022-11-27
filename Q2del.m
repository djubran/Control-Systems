
M1 = 6; 
M2 = 2;
B1 = 6;
B2 = 0.4;
K = 11;
L = 8;

A=[0 1 0 0; -K/M1 -B1/M1 K/M1 0; 0 0 0 1; K/M2 0 -K/M2 -B2/M2];
B=[0; 0; 0; 0];
C=[1 0 0 0];
D=L;
H=ss(A,B,C,D);
%2.3:
x02=[1 5 -1 5];
x01=[1 0 -1 0];
subplot(2,1,1)
initial(H,x01)
figure();
title('Output Response for x_0^1');


%% 2.4:
p=pole(H);
p4=[-0.3873 -0.3873 -0.3873 -0.3873];
L4=acker(A', C', p4);
Acl4=A-L4'*C;
H_fast2=ss(Acl4, L4', C, D);
%% 2.5:
p5=5*[-0.3873 -0.3873 -0.3873 -0.3873];
L5=acker(A', C', p5);
Acl5=A-L5'*C;
H_fast10=ss(Acl5, L5', C, D);
%% 2.6-7:
t = linspace(0,100,1000);
[y, tu, x] = initial(H, x01, t);
[y2, t2, x2] = lsim(H_fast2, y, t);
[y10, t10, x10] = lsim(H_fast10, y, t);
figure();
plot(t,x(:,1),t,x2(:,1),t,x10(:,1));
title('x_1 for Initial Conditions x_0^1 Without Noise');
legend('x1 original','x1 obsevred fast x2','x obsevred fast x10');
xlabel('time[sec]');
ylabel('x_1 [M]');
WN=randn(1,1000)'*sqrt(2);
[y2n, t2n, x2n] = lsim(H_fast2, y+WN,t);
[y10n, t10n, x10n] = lsim(H_fast10,y+WN, t);
figure();
plot(t,x(:,1),t,x2n(:,1),t,x10n(:,1));
title('x_1 for Initial Conditions x_0^1 With Noise');
legend('x1 original','x1 obsevred fast x2','x obsevred fast x10');
xlabel('time[sec]');
ylabel('x_1 [M]');