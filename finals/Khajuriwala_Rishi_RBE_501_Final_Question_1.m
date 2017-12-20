
%% Question 1 Torque equation 

syms m q l dq g Dq t th1(t) th2(t) 
% here in this question
% q = joint angle
% dq = q dot
% Dq = q double dot
%% End effector position
% here x(1) is actually l*cos(pi/2-q) some i have taken it as l*sinq 
x1 =[l*sin(q);
     -l*cos(q);];

%% Velocity
v = jacobian(x1,q)* (dq);
%% Kinetic Energy
k = 0.5 * m * (v.' * v);
%% Potential Energy
p = -1* m*g*l*cos(q);
%% lagrangian (L)
L = simplify(k-p)
%% euler- langrange equation
% tau =  d/dt(dL/d(dq)) -dL/dq
%% For the first part of the equation i.e d/dt(dL/d(dq))

dL_dq = diff(L,dq);
dL_dq_sub= subs(dL_dq,[q dq],[th1 diff(th1(t),t)]);
dL1 = diff(dL_dq_sub,t);
d_L_dq = subs(dL1,[th1 diff(th1(t),t) diff(th1(t),t,t)],[q dq Dq]);

%% for the second part of the equation i.e dL/dq
d_L_q = diff(L,q);
%% Tau
Tau = (d_L_dq - d_L_q)
%% Inertia matrix M
M = simplify(Tau - subs(Tau,Dq,0))/Dq
%% Gravity  G
G = simplify(subs(Tau, [dq Dq],[0 0]))
%% Coriolis Coupling Term
C =simplify( Tau- (M* (Dq).' + G))