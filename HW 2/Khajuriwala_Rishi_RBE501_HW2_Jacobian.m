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
d1= sym('d1','real');
d4= sym('d4','real');
d6= sym('d6','real');
a1= sym('a1','real');
a2= sym('a2','real');
a3= sym('a3','real');


link_number = [1,2,3,4,5,6]';
theta = [theta1,theta2,theta3,theta4,theta5,theta6]';
alpha = [pi/2,0,0,-pi/2,pi/2,0]';
d = [d1,0,0,d4,0,d6]';
a = [a1,a2,a3,0,0,0]';


DHParameters = [link_number,theta,d,alpha,a];
disp(DHParameters);

for n=1:3
    T(:,:,n) = Fk(theta(n),d(n),alpha(n),a(n));
end
T01 = T(:,:,1);
T12 = T(:,:,2);
T23 = T(:,:,3);



TT(:,:,1) = T(:,:,1);
for n=2:3
TT(:,:,n) = TT(:,:,n-1)*T(:,:,n);
end
T03 = simplify(TT(:,:,3));
T02 = simplify(TT(:,:,2));
TipPositionT03 = simplify(T03(1:3,4));
OrientationT03 = T03(1:3,1:3);
x = TipPositionT03(1);
y = TipPositionT03(2);
z = TipPositionT03(3);

x1 = (simplify(diff(x,theta1)));
y1 = simplify(diff(y,theta1));
z1 = simplify(diff(z,theta1));
x2 = simplify(diff(x,theta2));
y2 = simplify(diff(y,theta2));
z2 = simplify(diff(z,theta2));
x3 = simplify(diff(x,theta3));
y3 = simplify(diff(y,theta3));
z3 = simplify(diff(z,theta3));
k = [0,0,1]';
 RotationT01 = T01(1:3,1:3);
 RotationT02 = T02(1:3,1:3);
r2= RotationT01*k ;
r3= RotationT02*k;
xt = [5,5,10,0,0,0]';
j =[-sin(theta1)*(a1 + a3*cos(theta2 + theta3) + a2*cos(theta2)) -cos(theta1)*(a3*sin(theta2 + theta3) + a2*sin(theta2)) -a3*sin(theta2 + theta3)*cos(theta1)
    cos(theta1)*(a1 + a3*cos(theta2 + theta3) + a2*cos(theta2))  -sin(theta1)*(a3*sin(theta2 + theta3) + a2*sin(theta2)) -a3*sin(theta2 + theta3)*sin(theta1)
    0                                                            a3*cos(theta2 + theta3) + a2*cos(theta2)                a3*cos(theta2 + theta3)
    0                                                            sin(theta1)                                             sin(theta1)
    0                                                            -cos(theta1)                                            -cos(theta1)
    1                                                             0                                                      0]
% %  j= [x1 x2    x3
% %      y1 y2    y3
% %      z1 z2    z3
% %      0  r2(1) r3(1)
% %      0  r2(2) r3(2)
% %      1  r2(3) r3(3)]
size(j)
Jinverse = pinv(j)
size(Jinverse)
jointvelocity = vpa(subs(Jinverse*xt,[theta1,theta2,theta3,a1,a2,a3],[0.1973,2.6969,0.0,150,600,120]))
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

