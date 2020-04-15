%%Inverted Pendulum State Space Model Analysis
%%Ryan Fielding
clc;
clear all;
close all;
global A B K L
%% Constants
M = .081;   %kg, cart mass
m = 0.088;  %kg, total pend. mass
b = 10;    %N/m/s, cart friction estimate
g = 9.81;   %m/s^2
l = 0.10;   %m, pend. length to CM.

%calculate pend. inertia
m_rod = 0.035;  %kg, rod mass
m_disk = 0.053; %kg, disk mass
r_disk = 0.037; %m, disk radius

%%Small Wheel
% M = .081;   %kg, cart mass
% m = 0.069;  %kg, total pend. mass
% b = 10;    %N/m/s, cart friction estimate
% g = 9.81;   %m/s^2
% l = 0.10;   %m, pend. length to CM.
% 
% %calculate pend. inertia
% m_rod = 0.035;  %kg, rod mass
% m_disk = 0.034; %kg, disk mass
% r_disk = 0.02; %m, disk radius

I = (1/3)*m_rod*l^2 + 0.5*m_disk*r_disk^2 + m_disk*(2*l + r_disk)^2;  %kg.m^2

den = I*(M+m) + M*m*l^2; %denominator for the A and B matrices

%Matrices
A = [0      1              0           0;
     0 -(I+m*l^2)*b/den  (m^2*g*l^2)/den   0;
     0      0              0           1;
     0 -(m*l*b)/den       m*g*l*(M+m)/den  0];
B = [     0;
        I/den;
          0;
        m*l/den];
C = [1 0 0 0;
     0 0 1 0]; %x and theta are measurable states (potentiometer and optical encoder)
D = [0;
     0];


%% State space modelling
states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'u'};
outputs = {'x'; 'phi'};

sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
sys_tf = tf(sys_ss);
polesOL = eig(A); %one in the right half plane, thus unstable

co = ctrb(sys_ss);
ob = obsv(sys_ss);
controllability = rank(co); %rank of 4 = num of states, thus controllable.
observability = rank(ob); %rank of 4 = num states, thus observable.


%Convert to Discrete Time SS Model
Ts = 1/100; %Controller runs at 100Hz
sys_d = c2d(sys_ss,Ts,'zoh');
A = sys_d.a;
B = sys_d.b;
C = sys_d.c;
D = sys_d.d;

%state feedback test
%poles1 = [ -10 + 5i -10-5i -4 -5]; %choose desired poles
%k = acker(A, B, poles1);
%eig(A-B*k); %check system poles

%% Digital LQR Controller

Q = C'*C;
Q(1,1) = 1000; %x
%Q(2,2) = 1; %x
Q(3,3) = 5000; %theta
R = 1;

%must use dlqr for discrete time LQR controller
[K,S,P_Sys] = dlqr(A,B,Q,R);
P_Sys
K


% instead of LQR, try
% P_Sys = [-0.1 + 0.5i, -0.1 - 0.5i, 0.1, 0.2];
% K = place(A,B,P_Sys)
SFS = idss(A-B*K,B,C,D,'Ts',Ts);
%Examine the pole-zero map.
pzmap(SFS)

%model and simulate LQR controller on system
Ac = [(A-B*K)];
Bc = [B];
Cc = [C];
Dc = [D];

states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'r'};
outputs = {'x'; 'phi'};

sys_cl = ss(Ac,Bc,Cc,Dc,Ts,'statename',states,'inputname',inputs,'outputname',outputs);

figure;
t = 0:0.01:5;
r =0.2*ones(size(t));
[y,t,x]=lsim(sys_cl,r,t);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','cart position (m)')
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
title('Step Response with Digital LQR Control')

%% Observer based control
%Observer poles should be ~5-10 times the system LQR ctrl poles.
P_Obsv = 5*P_Sys;

L = place(A',C',P_Obsv)';

%% Model and simulate observer feedback whole system
Ace = [(A-B*K) (B*K);
       zeros(size(A)) (A-L*C)];
Bce = [B;
       zeros(size(B))];
Cce = [Cc zeros(size(Cc))];
Dce = [0;0];

states = {'x' 'x_dot' 'phi' 'phi_dot' 'e1' 'e2' 'e3' 'e4'};
inputs = {'r'};
outputs = {'x'; 'phi'};

sys_est_cl = ss(Ace,Bce,Cce,Dce,'statename',states,'inputname',inputs,'outputname',outputs);
t = 0:0.01:10;
%to step input 0.2m
r = -2*ones(size(t));

% 
% f = figure('units','inch','position',[4,4,12,6]);
% subplot(1,2,1);
% [y,t,x]=lsim(sys_est_cl,r,t);
% [AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
% set(get(AX(1),'Ylabel'),'String','cart position (m)')
% set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
% %title('Step Response with Observer-Based State-Feedback Control')
% hold on
% grid on
% legend('x', 'theta');
% 
% %plot dot states
% subplot(1,2,2)
% [AX2,H3,H4] = plotyy(t,x(:,2),t,x(:,4),'plot');
% set(get(AX2(1),'Ylabel'),'String','cart velocity (m/s)')
% set(get(AX2(2),'Ylabel'),'String','pendulum angular vel (rad/s)')
% %title('Step Response with Observer-Based State-Feedback Control')
% hold on
% grid on
% legend('x dot', 'theta dot');
% 
% saveas(f,'models.png');
% 
% f2 = figure('units','inch','position',[4,4,12,6]);
% %subplot(1,2,1);
% %[y,t,x]=lsim(sys_est_cl,r,t);
% [AX,H1,H2] = plotyy(t,x(:,5),t,x(:,7),'plot');
% %set(get(AX(1),'Ylabel'),'String','cart position (m)')
% %set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
% %title('Step Response with Observer-Based State-Feedback Control')
% hold on
% grid on
% %legend('x', 'theta');

%% Observer tuning
% 
Ao = [A-L*C];
Bo = [B];
Co = [C];
Do = [D];
% states = {'x' 'x_dot' 'phi' 'phi_dot'};
% inputs = {'r'};
% outputs = {'x hat'; 'phi hat'};

% sys_obsv = ss(Ao,Bo,Co,Do,'statename',states,'inputname',inputs,'outputname',outputs);
% f2 = figure('units','inch','position',[4,4,12,6]);
% [y,t,x]=lsim(sys_obsv,r,t);
% [AX3,H13,H23] = plotyy(t,y(:,1),t,y(:,2),'plot');
% set(get(AX3(1),'Ylabel'),'String','cart position (m)')
% set(get(AX3(2),'Ylabel'),'String','pendulum angle (radians)')
% %title('Step Response with Observer-Based State-Feedback Control')
% hold on
% grid on
% legend('x hat', 'theta hat');

%% Save matrices to .h file for tiva
exportToC();
copyfile obsv.h '/Users/Ryan/Repos/inverted-pendulum/tiva/'

