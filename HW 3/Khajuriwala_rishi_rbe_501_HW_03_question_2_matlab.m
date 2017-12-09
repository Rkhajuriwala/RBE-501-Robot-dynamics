clear all
close all
clc
%% QUESTION 2
%% Terms used in this code
% vi = velocity (i=1,2)
% ki= kinetic energy (i=1,2)
% pi = potential energy (i=1,2)
%% Joint angles
q1= sym('q1','real'); 
q2= sym('q2','real'); 
syms q1(t) q2(t) real;
%% Link lengths (link lenghts divided by 2)
a1= sym('a1','real'); 
a2= sym('a2','real');
%% Point masses of the links
m1= sym('m1','real');
m2= sym('m2','real');
%% qi_dot(i= 1,2)
dq1= sym('dq1','real');
dq2= sym('dq2','real');
%% qi_double_dot(i= 1,2)
Dq1= sym('Dq1','real');
Dq2= sym('Dq2','real');
%% Gravity 
g= sym('g','real');
%% Moment of inertia
I1 =sym('I1','real');
I2 =sym('I2','real');
%%extra
t = sym('t','real');
syms t1 t2  t1(t) t2(t) th1 th2 dth1 dth2 ddth1 ddth2 real;
assume(t1(t),'real');
assume(t2(t),'real');
%% Coorrdinates of link 1
x1 = 0.5*a1*cos(q1);
y1 = 0.5*a1*sin(q1);
%% Velocities for link 1
v1_x = diff(x1,t);
v1_y = diff(y1,t);
v1 = [v1_x; v1_y];
%% Coorrdinates of link 1
x2 = a1*cos(q1) + 0.5*a2*cos(q1 + q2);
y2 = a1*sin(q1) + 0.5*a2*sin(q1 + q2);
%% Velocities for link 2
v2_x = diff(x2,t);
v2_y = diff(y2,t);
v2 = [v2_x; v2_y];
%% Kinetic Energies
k1 = simplify((0.5*m1*(v1.' * v1)) + (0.5*I1*(diff(q1(t),t)^2)));
k2 = simplify((0.5*m2*(v2.' * v2)) + (0.5*I2*(diff(q1(t),t) + diff(q2(t),t))^2));


%% Total Kinetic Energy k
k = k1 + k2;
%% Potential Energies
p1 = 0.5*m1*g*a1*sin(q1);
p2 = m2*g*a1*sin(q1) + 0.5*g*a2*m2*sin(q1 + q2);
%% Total Potential Energy p
p = p1 + p2 ;
%% Lagragian L
L = simplify(k - p);
%% euler- langrange equation
% tau =  d/dt(dL/d(dq)) -dL/dq

%% for tau1
% for the first part of the equation i.e d/dt(dL/d(dq))
L_1 = subs(L, diff(q1(t),t), dth1);
d_L_dq_1= diff(L_1,dth1);
L_dq_1_t = subs(d_L_dq_1,[dth1],[diff(q1(t),t)]);
d_L_dq_1_t = diff(L_dq_1_t,t);
d_L_dq_1 = subs(d_L_dq_1_t, [q1 q2 diff(q1(t),t) diff(q2(t),t) diff(q1(t),t,t) diff(q2(t),t,t)],[th1 th2 dth1 dth2 ddth1 ddth2]);
% for the second part of the equation i.e dL/dq
L_q1 = subs(L,[q1(t)],[th1]);
d_L_q1_1 = diff(L_q1, th1);
d_L_q1 = subs(d_L_q1_1, [q1 q2], [th1 th2]);
% tau1
Tau_1 = simplify(d_L_dq_1 - d_L_q1)
%% for tau2
% for the first part of the equation i.e d/dt(dL/d(dq))
L_2 = subs(L, diff(q2(t),t), dth2);
d_L_dq_2 = diff(L_2,dth2);
L_dq_2_t = subs(d_L_dq_2,[dth2],[diff(q2(t),t)]);
d_L_dq_2_t= diff(L_dq_2_t,t);
d_L_dq_2= subs(d_L_dq_2_t, [q1 q2 diff(q1(t),t) diff(q2(t),t) diff(q1(t),t,t) diff(q2(t),t,t)],[th1 th2 dth1 dth2 ddth1 ddth2]);
% for the second part of the equation i.e dL/dq
L_q2= subs(L, q2(t), th2);
d_L_q2_1= diff(L_q2, th2);                              
d_L_q2 = subs(d_L_q2_1, [q1 q2], [th1 th2]);

% tau2
Tau_2 = simplify(d_L_dq_2 - d_L_q2)