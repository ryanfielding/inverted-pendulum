%%Inverted Pendulum State Space Model Analysis
%%Ryan Fielding
clc;
clear all;
close all;

%% Constants
M = .081;   %kg, cart mass
m = 0.088;  %kg, total pend. mass
b = 0.1;    %N/m/s, cart friction estimate
g = 9.81;   %m/s^2
l = 0.10;   %m, pend. length to CM.

%calculate pend. inertia
m_rod = 0.035;  %kg, rod mass
m_disk = 0.053; %kg, disk mass
r_disk = 0.037; %m, disk radius
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


%% State modelling
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

%state feedback test
%poles1 = [ -10 + 5i -10-5i -4 -5]; %choose desired poles
%k = acker(A, B, poles1);
%eig(A-B*k); %check system poles

%% LQR Controller

Q = C'*C;
%Weights on errors for cart x, pend. theta.
Q(1,1) = 1;
Q(3,3) = 10;
R = 1;
[K,S,P_Sys] = lqr(A,B,Q,R);

%model and simulate LQR controller on system
Ac = [(A-B*K)];
Bc = [B];
Cc = [C];
Dc = [D];

states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'r'};
outputs = {'x'; 'phi'};

sys_cl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);

%% Observer based control
%Observer poles should be ~5-10 times the system LQR ctrl poles.
P_Obsv = 5*P_Sys;

L = place(A',C',P_Obsv)';

%display results for TIVA
A
B
C
D
K
L
A - B*K

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
% 
t = 0:0.01:5;
%to step input 0.2m
r = -0.2*ones(size(t));

[y,t,x]=lsim(sys_est_cl,r,t);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','cart position (m)')
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
title('Step Response with Observer-Based State-Feedback Control')
hold on
%plot xdot, theta dot
plot(t,x(:,2), 'g');
plot(t,x(:,4));