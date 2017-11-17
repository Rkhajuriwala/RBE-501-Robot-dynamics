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

%Question 2 -->defining DH parameters for LR Mate 200iC
link_number = [1,2,3,4,5,6]';
theta = [theta1,(theta2+90)*deg2rad,theta3,theta4,theta5,theta6]';
d = [330,0,0,320,0,80]';
alpha = [pi/2,0,pi/2,-pi/2,pi/2,0]';
a = [75,300,75,0,0,0]';

DHParameters = [link_number,theta,d,alpha,a];
disp(DHParameters)
% Question 3 The Frame Transformations are listed below:
for n=1:6
    T(:,:,n) = Fk(theta(n),d(n),alpha(n),a(n));
end
T01 = T(:,:,1)
T12 = T(:,:,2)
T23 = T(:,:,3)
T34 = T(:,:,4)
T45 = T(:,:,5)
T56 = T(:,:,6)


%Question 4 The Composite Transformation 
TT(:,:,1) = T(:,:,1);
for n=2:6
TT(:,:,n) = TT(:,:,n-1)*T(:,:,n);
end
T06 = simplify(TT(:,:,6))

%Question 5  Position and Orientation Forward kinematics by solving composite Transformation
TipPositionT06 = T06(1:3,4)
OrientationT06 = T06(1:3,1:3)

% Position and Orientation Forward kinematics by giving joint angles [0 0 0 0 0 0]
Thome = double(subs(TT(:,:,6),[theta1,theta2,theta3,theta4,theta5,theta6],[0 0 0 0 0 0]));
TipPostionThome = Thome(1:3,4)
OrientationThome =Thome(1:3,1:3)
Thome = plotarm(0,0,0,0,0,0) %3D plot for this position

% Question 7 Position and Orientation Forward kinematics by giving joint angles [0 75 30 135 -45 60]
T_Angles=double(subs(TT(:,:,6),[theta1,theta2,theta3,theta4,theta5,theta6],[0*deg2rad,75*deg2rad,30*deg2rad,135*deg2rad,-45*deg2rad,60*deg2rad]));
TipPostionT_Angles = T_Angles(1:3,4)
OrientationT_Angles =T_Angles(1:3,1:3)
T_Angles = plotarm(0,75,30,135,-45,60) %3D plot for the given joint angles



%function created for transform matrix
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

%function for 3D plot
function [Tfinal] = plotarm(t1, t2, t3, t4, t5, t6 )
jointAngleInDegree = [t1, t2, t3, t4, t5, t6];
theta = [jointAngleInDegree(1),jointAngleInDegree(2)+90,jointAngleInDegree(3), jointAngleInDegree(4),jointAngleInDegree(5), jointAngleInDegree(6)]';
d = [330,0,0,320,0,80]';
alpha = [pi/2,0,pi/2,-pi/2,pi/2,0]';
a = [75,300,75,0,0,0]';
for n=1:6
    T(:,:,n) = Fk(theta(n),d(n),alpha(n),a(n));
end
TT(:,:,1) = T(:,:,1);
for n=2:6
    TT(:,:,n) = TT(:,:,n-1)*T(:,:,n);
end
xJ=[0,0,0,0,0,0,0];
yJ=[0,0,0,0,0,0,0];
zJ=[0,0,0,0,0,0,0];
for n=1:6
    xJ(n+1) = TT(1,4,n);
    yJ(n+1) = TT(2,4,n);
    zJ(n+1) = TT(3,4,n);
end
figWidth = 600; 
figHeight = 450;
rect = [0 50 figWidth figHeight];
figure('OuterPosition',rect)
plot3(xJ,yJ,zJ,'b','LineWidth',5);
hold on
scatter3(xJ,yJ,zJ,'filled','SizeData',100);
quiver3(xJ(end), yJ(end), zJ(end), TT(1,1,6), TT(2,1,6),TT(3,1,6),500,'g','LineWidth',2)
quiver3(xJ(end), yJ(end), zJ(end), TT(1,2,6), TT(2,2,6),TT(3,2,6),500,'b','LineWidth',2)
quiver3(xJ(end), yJ(end), zJ(end), TT(1,3,6), TT(2,3,6),TT(3,3,6),500,'r','LineWidth',2)
view(6.5651,34.0948)
axis([-1000 1000 -1000 1000 0 1500]);
grid on
title('3D plot of FANUC LRMate200iC','FontSize',14)
xlabel('x-axis (mm)','FontSize',20)
ylabel('y-axis (mm)','FontSize',20)
zlabel('z-axis (mm)','FontSize',20)
hold off
Tfinal = TT(:,:,6);
end


