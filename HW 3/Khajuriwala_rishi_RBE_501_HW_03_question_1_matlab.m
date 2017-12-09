clear all
close all
clc
%% QUESTION 1
%% terms used in this code
% vi = velocity (i=1,2,3)
% ki= kinetic energy (i=1,2,3)
% pi = potential energy (i=1,2,3)
%% joint angles
q1= sym('q1','real'); 
q2= sym('q2','real'); 
q3= sym('q3','real'); 
%% link lengths
l1= sym('l1','real'); 
l2= sym('l2','real');
l3= sym('l3','real');
%% point masses of the links
m1= sym('m1','real');
m2= sym('m2','real');
m3= sym('m3','real');
%% qi_dot(i= 1,2,3)
dq1= sym('dq1','real');
dq2= sym('dq2','real');
dq3= sym('dq3','real');
%% qi_double_dot(i= 1,2,3)
Dq1= sym('Dq1','real');
Dq2= sym('Dq2','real');
Dq3= sym('Dq3','real');
%% gravity 
g= sym('g','real');
%%
t = sym('t','real');
syms t1 t2 t2 t1(t) t2(t) t3(t) real;
assume(t1(t),'real');
assume(t2(t),'real');
assume(t3(t),'real');

%% DH parameters 

theta = [q1 q2 q3]';
alpha = [sym(pi/2) 0 0]';
a = [0 l2 l3]';
d = [l1 0 0]';

T1 = [cos(q1)    0      sin(q1)    0;
      sin(q1)    0     -cos(q1)    0;
      0          1            0   l1;
      0          0            0   1];
  
T2 = [cos(q2)    -sin(q2)    0    l2*cos(q2);
      sin(q2)    cos(q2)     0    l2*sin(q2);
      0          0           1    0;
      0          0           0    1];
  
T3 = [cos(q3)    -sin(q3)    0    l3*cos(q3);
      sin(q3)    cos(q3)     0    l3*sin(q3);
      0          0           1    0;
      0          0           0    1];
 
T01 = T1;
T02 = T01*T2;
T03 = simplify(T02*T3);
%% velocities 
v1 = jacobian(T01(1:3,4),[q1])* [dq1];
v2 = jacobian(T02(1:3,4),[q1, q2]) * [dq1; dq2];
v3 = jacobian(T03(1:3,4),[q1, q2, q3]) * [dq1; dq2; dq3];
%% kinetic energies
k1 = 0.5 * m1 * (v1.' * v1);
k2 = 0.5 * m2 * (v2.' * v2);
k3 = 0.5 * m3 * (v3.' * v3);
%% potential energies
p1 = m1 * g * T01(3,4);
p2 = m2 * g * T02(3,4);
p3 = m3 * g * T03(3,4);
%% Total kinetic energy (k)
k = k1 + k2 + k3;
%% Total potential energy (p)
p = p1 + p2 + p3;
%% lagrangian (L)
L = simplify(k - p);
%% euler- langrange equation
% tau =  d/dt(dL/d(dq)) -dL/dq

%% for tau1
% for the first part of the equation i.e d/dt(dL/d(dq))

d_L_dq_1 = diff(L,dq1);
L_dq_1_t = subs(d_L_dq_1, [q1 q2 q3 dq1 dq2 dq3], [t1 t2 t3 diff(t1(t),t) diff(t2(t),t) diff(t3(t),t)]);
d_L_dq_1_t = diff(L_dq_1_t, t);
d_L_dq_1 = subs(d_L_dq_1_t, [t1 t2 t3 diff(t1(t),t) diff(t2(t),t) diff(t3(t),t) diff(t1(t),t,t) diff(t2(t),t,t) diff(t3(t),t,t)], [q1 q2 q3 dq1 dq2 dq3 Dq1 Dq2 Dq3]);

% for the second part of the equation i.e dL/dq
d_L_q1 = diff(L,q1);
% tau1
Tau_1 = simplify(d_L_dq_1 - d_L_q1)
%% for tau2
% for the first part of the equation i.e d/dt(dL/d(dq))

d_L_dq_2 = diff(L,dq2);
L_dq_2_t = subs(d_L_dq_2, [q1 q2 q3 dq1 dq2 dq3], [t1 t2 t3 diff(t1(t),t) diff(t2(t),t) diff(t3(t),t)]);
d_L_dq_2_t = diff(L_dq_2_t, t);
d_L_dq_2 = subs(d_L_dq_2_t, [t1 t2 t3 diff(t1(t),t) diff(t2(t),t) diff(t3(t),t) diff(t1(t),t,t) diff(t2(t),t,t) diff(t3(t),t,t)], [q1 q2 q3 dq1 dq2 dq3 Dq1 Dq2 Dq3]);

% for the second part of the equation i.e dL/dq
d_L_q2 = diff(L,q2);
% tau2
Tau_2 = simplify(d_L_dq_2 - d_L_q2)
%% for tau3
% for the first part of the equation i.e d/dt(dL/d(dq))

d_L_dq_3 = diff(L,dq3);
L_dq_3_t = subs(d_L_dq_3, [q1 q2 q3 dq1 dq2 dq3], [t1 t2 t3 diff(t1(t),t) diff(t2(t),t) diff(t3(t),t)]);
d_L_dq_3_t = diff(L_dq_3_t, t);
d_L_dq_3 = subs(d_L_dq_3_t, [t1 t2 t3 diff(t1(t),t) diff(t2(t),t) diff(t3(t),t) diff(t1(t),t,t) diff(t2(t),t,t) diff(t3(t),t,t)], [q1 q2 q3 dq1 dq2 dq3 Dq1 Dq2 Dq3]);

% for the second part of the equation i.e dL/dq
d_L_q3 = diff(L,q3);
% tau1
Tau_3 =simplify(d_L_dq_3 - d_L_q3)


%% QUESTION 2