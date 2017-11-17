% % fk
clear all
close all
clc
theta1= sym('theta1','real');
theta2= sym('theta2','real');
theta3= sym('theta3','real');
theta4= sym('theta4','real');
theta5= sym('theta5','real');
theta6= sym('theta6','real');
pi = sym('pi');
deg2rad=pi/180;
r2d=180/pi;

link_number = [1,2,3,4,5,6]';
theta = [theta1,theta2,theta3,theta4,theta5,theta6]';
d = [475,0,0,720,0,85]';
alpha = [pi/2,0,0,-pi/2,pi/2,0]';
a = [150,600,120,0,0,0]';
a1=150;
a2=600;
a3=290;
d1=475;



%% For inverse kinematics

for n=1:6
    T(:,:,n) = Fk(theta(n),d(n),alpha(n),a(n));
end
T01 = T(:,:,1);
T12 = T(:,:,2);
T23 = T(:,:,3);
T34 = T(:,:,4);
T45 = T(:,:,5);
T56 = T(:,:,6);

TT(:,:,1) = T(:,:,1);
for n=2:6
TT(:,:,n) = TT(:,:,n-1)*T(:,:,n);
end
T06 = simplify(TT(:,:,6));

TipPositionT06 = T06(1:3,4);
Tipx = TipPositionT06(1);
Tipy = TipPositionT06(2);
Tipz = TipPositionT06(2);
OrientationT06 = T06(1:3,1:3);
% % t= [r11 r12 r13 px 
% %     r21 r22 r23 py
% %     r31 r32 r33 pz
% %     0   0   0   1];
t= [1 0 0 500
    0 1 0 100
    0 0 1 1500
    0 0 0 1];

Tipsample= t(1:3,4);
Tipx = Tipsample(1);
Tipy = Tipsample(2);
Tipz = Tipsample(3);
%% for theta 1
l = vpa(atan2(Tipy,Tipx));


 %% for theta 3
D = (Tipx^2 + Tipy^2 -(a1)^2 +(Tipz-d1)^2- a2^2- a3^2)/(2*a2*a3);
dsine= real(sqrt(1-(D)^2));
f= vpa(atan2(dsine,D));


%% for theta 2
e = vpa(atan2(Tipz-d1,sqrt(Tipx^2+Tipy^2- a1^2))- atan2(a3*sin(f),a2+a3*cos(f)))+1.57;

%%

T36=simplify(T34*T45*T56);
T03 = T01*T12*T23;
T03inverse=inv(T03);
T36third = T36(1:4,3);
q = subs(simplify(T03inverse*t),[theta1,theta2,theta3],[l,e,f]);
q2 =(simplify((q(1:4,3))));

%% for theta5
c5 = q2(3);
s5 = sqrt(1-c5^2);
v = vpa(atan2(s5,c5));

%% for theta4
c4= q2(1);
s4 = q2(2);
o = vpa(atan2(s4,c4));

%% for theta6
q3 = simplify(q(3,1:4));
c6 = q3(1);
s6 = q3(2);
u = vpa(atan2(s6,-c6));
%% All Theta values 
theta1 = (l)%*r2d
theta2 = (e)%*r2d
theta3 = (f)%*r2d
theta4 = (o)%*r2d
theta5 = (v)%*r2d
theta6 = (u)%*r2d
%%  Function for Fk
function [ transMatrix ] =Fk(theta,d,alpha,a)
    rotOldZAxis = [cos(theta) -sin(theta) 0 0;...
    sin(theta) cos(theta) 0 0;...
    0 0 1 0;...
    0 0 0 1];
    translationOldZAxis = [1 0 0 0;...
    0 1 0 0;...
    0 0 1 d;...
    0 0 0 1];
    translationNewXAxis = [1 0 0 a;...
    0 1 0 0;...
    0 0 1 0;...
    0 0 0 1];
    rotNewXAxis = [1 0 0 0;...
    0 cos(alpha) -sin(alpha) 0;...
    0 sin(alpha) cos(alpha) 0;...
    0 0 0 1];

    transMatrix = rotOldZAxis*translationOldZAxis*translationNewXAxis*rotNewXAxis;
end

