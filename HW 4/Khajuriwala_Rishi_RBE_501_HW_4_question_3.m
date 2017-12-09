%% QUESTION 3

clc
clear all;
close all;

%% A-->Constraint Equations:



%% variables
syms a c r r_SW
% For left wheel
al = pi/2;
bl = 0;
l = a/2;

% For right wheel
ar = -pi/2;
br = pi;
% l = a/2;

% For castor omniwheel
ao = pi;
bo = 0;
lc = c;
gamma = 0;

%% B-->Constraint Matrix:
% left rolling and sliding
rol_l = [sin(al+bl) -cos(al+bl) -l*cos(bl)]
sli_l = [cos(al+bl) sin(al+bl) -l*sin(bl)]

% right rolling and sliding
rol_r = [sin(ar+br) -cos(ar + br) -l*cos(br)]
sli_r = [cos(ar+ br) sin(ar+br) -l*sin(br)]

% castor omniwheel rolling and sliding
rol_C = [sin(ao+bo+gamma) -cos(ao + bo +gamma) -lc*cos(bo+gamma)]
sli_C = [cos(ao+bo+gamma) sin(ao + bo+gamma) -lc*sin(bo+gamma)]

% Combining all the rolling and sliding 
J_rol = vpa([rol_l;rol_r;rol_C])
C_sli = vpa([sli_l;sli_r;sli_C])

J2 = [r, 0, 0; 0, r, 0; 0, 0, r]
C2 = [0, 0, 0; 0, 0, 0; 0, 0, r_SW]




% just writing the equations again by removing the redundant constraints and removing the third omniwheel as it
% doesnot have any constraints from the slide number 155
Simpli_Constraints = [rol_l;rol_r;sli_l]
JJ2 = [r, 0, 0; 0, r, 0; 0, 0, 0]

%% C--> Mobile kinematics

syms alpha wL wR wO

Rrw = [cos(alpha) sin(alpha) 0;
        -sin(alpha) cos(alpha) 0;
         0          0          1];
     

vel_w = inv(Rrw)*inv(Simpli_Constraints)*JJ2*[wL;wR;wO]


%% D--> Position Kinematics
syms x_R y_R theta d 

% Transformation matrix between the world frame and the robot frame
TRT = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
    sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
    0 sin(alpha) cos(alpha) d;
    0 0 0 1];

TWR = [cos(alpha) -sin(alpha) 0 x_R;
       sin(alpha) cos(alpha) 0 y_R;
       0 0 1 0;
       0 0 0 1];


TWT = TWR*TRT



%% F--> Combined Jacobian
vel_w_b = [vel_w(1);vel_w(2);0;0;0;vel_w(3)];

%Calculating the jacobian of the base from the wheel velocity
J_base = simplify([diff(vel_w_b, wL), diff(vel_w_b,wR)]);


% For the tip jacobian wrt to the world frame we find by the following way

% For the translational component of the Jacobian we have
syms X1 X2 X3 Y1 Y2 Y3 theta1 theta2 theta3
% Assume we have 6x3 Velocity Jacobian from TRT
J_arm = [X1 X2 X3;
         Y1 Y2 Y3;
         0 0 0;
         0 0 0;
         0 0 0;
         1 1 1];

Jtip = [J_arm J_base]
%% E-->Combined Velocity Kinematics
Tip_Velocity = Jtip * [theta1; theta2; theta3; wL;wR]
%% G--> Force Propagation
syms fx fy fz nx ny nz
F = [fx; fy; fz; nx; ny; nz];
Torque = Jtip' * F;

Wheel_T_1 = Torque(4)
Wheel_T_2 = Torque(5)
