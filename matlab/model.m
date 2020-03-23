%%Inverted Pendulum State Space Model Analysis
%%Ryan Fielding
clc;
clear all;
close all;

%% Constants
M = .3;         %kg, cart mass
m = 0.1;        %kg, rod mass
b = 0.1;        %N/m/s, cart friction
g = 9.8;        %m/s^2
l = 0.15;        %m, pend. length to CM.

m_disk = 0.3;   %kg, end of rod mass
r_disk = 0.05;  %cm
%calculate pend. inertia
I = (1/3)*m*l^2 + 0.5*m_disk*r_disk^2 + m_disk*(l + r_disk)^2;  %kg.m^2

den = I*(M+m) + M*m*l^2; %denominator for the A and B matrices

%Matrices
A = [0      1              0           0;
     0 -I*b/den  (m^2*g*l^2)/den   0;
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
poles1 = [ -10 + 5i -10-5i -4 -5]; %choose desired poles
k = acker(A, B, poles1);
Anew = A - B*k;
eig(Anew); %check system poles

%% LQR

Q = C'*C;
Q(1,1) = 5000;
Q(3,3) = 100;
R = 1;
K = lqr(A,B,Q,R);

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
poles = eig(Ac);

P = [-40 -41 -42 -43];
L = place(A',C',P)';

%results for TIVA
A
B
C
D
K
L
A - B*K
%model and simulate observer feedback whole system
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
% t = 0:0.01:5;
% %to step input 0.2m
% r = -0.2*ones(size(t));
% 
% [y,t,x]=lsim(sys_est_cl,r,t);
% [AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
% set(get(AX(1),'Ylabel'),'String','cart position (m)')
% set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
% title('Step Response with Observer-Based State-Feedback Control')
