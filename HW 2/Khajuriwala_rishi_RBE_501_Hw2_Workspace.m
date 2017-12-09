%% workspace top view
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
syms r11 r12 
syms r13 r21 
syms r22 r23 
syms r31 r32 
syms r33 px 
syms py pz


link_number = [1,2,3,4,5,6]';
theta = [theta1,theta2,theta3,theta4,theta5,theta6]';
% d = [0.475,0,0,0.720,0,0.085]';
 d = [475,0,0,720,0,85]';
alpha = [0,pi/2,0,pi/2,-pi/2,pi/2]';
%  a = [0.150,0.600,0.120,0,0,0]';
a = [0,150,600,120,0,0]';
a1=0;
a2=150;
a3=600;
d1=475;
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

TipPositionT03 = T03(1:3,4);
OrientationT03 = T03(1:3,1:3);
% Question(a)For workspace visualization 
x = TipPositionT03(1)
y = TipPositionT03(2)
z = TipPositionT03(3)
p = [];
index = 0;
for theta1 = -pi/2:0.1:pi/2
    for theta2 = -pi/3:0.1:pi/3
        for theta3 = -pi/3:0.1:pi/3
            index = index +1;
            x = 30*cos(theta1)*(4*cos(theta2 + theta3) + 20*cos(theta2) + 5);
            y = 30*sin(theta1)*(4*cos(theta2 + theta3) + 20*cos(theta2) + 5);
            z = 120*sin(theta2 + theta3) + 600*sin(theta2) + 475;
            p(index,:) = [x,y,z];
        end
     end
end
plot3(p(:,1),p(:,2),p(:,3),'*b')

figure;
%%  workspace side view
hold on
theta1 = pi/2:-1*pi/400:-pi/6;
x1 = 1320*cos(theta1);
y1 = 1320*sin(theta1) +475;

theta2 = pi/2:-1*pi/400:-pi/6;
x2 = 720*cos(theta2);
y2 = 720*sin(theta2)+1075;

theta3 = -pi/6:-1*pi/400:-(5*pi)/6;
x3 = 600*cos(theta3);
y3 = 600*sin(theta3)+475;

theta4 = 210*pi/180:1*pi/400:330*pi/180;
x4 = 720*cos(theta4) + 600*sin(pi/3);
y4 = 720*sin(theta4) + 475-600*sin(pi/6);
plot(x1,y1,x2,y2,x3,y3,x4,y4)
hold off

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
