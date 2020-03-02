%%Inverted Pendulum State Space Model
%%Ryan Fielding
clc;
clear all;
close all;

%% Constants
M = .5;
m = 0.2;
b = 0.1;
I = 0.006;
g = 9.8;
l = 0.3;

den = I*(M+m)+M*m*l^2; %denominator for the A and B matrices

A = [0      1              0           0;
     0 -(I+m*l^2)*b/den  (m^2*g*l^2)/den   0;
     0      0              0           1;
     0 -(m*l*b)/den       m*g*l*(M+m)/den  0];
B = [     0;
     (I+m*l^2)/den;
          0;
        m*l/den];
C = [1 0 0 0;
     0 0 1 0];
D = [0;
     0];

states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'u'};
outputs = {'x'; 'phi'};

sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs)
sys_tf = tf(sys_ss)
co = ctrb(sys_ss);
controllability = rank(co) %rank of 4 = num of states, thus controllable.

