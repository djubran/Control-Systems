% id : 207110891 , first digit 2 ; second digit 0
%%Part A
M = 0.5+0.1*2;
m = 0.2;
b = 0.1;
I = 0.006;
g = 9.8;
l = 0.3 + 0.1*0;

p = I*(M+m)+M*m*l^2; %denominator for the A and B matrices

A = [0      1              0           0;
     0 -(I+m*l^2)*b/p  (m^2*g*l^2)/p   0;
     0      0              0           1;
     0 -(m*l*b)/p       m*g*l*(M+m)/p  0];
B = [     0;
     (I+m*l^2)/p;
          0;
        m*l/p];
C = [1 0 0 0;
     0 0 1 0];
D = [0;
     0];

states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'u'};
outputs = {'x'; 'phi'};


sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

poles = eig(A);

co = ctrb(sys_ss);
controllability = rank(co);
Q = 1*eye(4);
Q(1,1) = 5000;
Q(3,3) = 100;
R = 1;
K = lqr(A,B,Q,R);

Ac = [(A-B*K)];
Bc = [B];
Cc = [C];
Dc = [D];
new_poles = eig(Ac);


states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'r'};
outputs = {'x'; 'phi'};

Cn = [1 0 0 0];
sys_ss = ss(A,B,Cn,0);
Nbar = rscale(sys_ss,K);


sys_cl = ss(Ac,Bc*Nbar,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);

t = 0:0.01:5;
r =ones(size(t));
[y,t,x]=lsim(sys_cl,r,t);
figure(1);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','cart position (m)')
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
title('Step Response LQR Control after fixing steady-state error')

step_info = lsiminfo(y,t);
cart_info = step_info(1);
pend_info = step_info(2);
%%Part B
ob = obsv(sys_ss);
observability = rank(ob);

Obv_poles = [-40 -41 -42 -43];
L = place(A',C',Obv_poles)';

Ace = [(A-B*K) (B*K);
       zeros(size(A)) (A-L*C)];
Bce = [B*Nbar;
       zeros(size(B))];
Cce = [Cc zeros(size(Cc))];
Dce = [0;0];

states = {'x' 'x_dot' 'phi' 'phi_dot' 'e1' 'e2' 'e3' 'e4'};
inputs = {'r'};
outputs = {'x'; 'phi'};
eig_new=eig(Ace);
sys_est_cl = ss(Ace,Bce,Cce,Dce,'statename',states,'inputname',inputs,'outputname',outputs);

t = 0:0.01:5;
r = ones(size(t));
[y_obv,t,x_obv]=lsim(sys_est_cl,r,t);
figure(2);
[AX,H1,H2] = plotyy(t,y_obv(:,1),t,y_obv(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','cart position (m)')
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
title('Step Response with Observer-Based State-Feedback Control')
step_est_info = lsiminfo(y_obv,t);
cart_est_info = step_est_info(1);
pend_est_info = step_est_info(2);
% estimated vector state vs real vector state
figure(3);
subplot(1,2,1)
plot(t,x(:,:));
title('actual state vector');
legend('x' ,'x''', 'phi', 'phi''');
xlabel('time[sec]');
ylabel('states');
subplot(1,2,2)
plot(t,x_obv(:,:));
title('estimated state vector');
legend('x' ,'x''', 'phi' ,'phi''', 'e1', 'e2' ,'e3', 'e4');
xlabel('time[sec]');
ylabel('states');
%poles diffferent
x0 = [0.1 0 0 0];
[y_obv_new,t,x_obv]=lsim(sys_est_cl,r,t,[x0 x0])
figure(9)
plot(t,y_obv_new(:,:));

