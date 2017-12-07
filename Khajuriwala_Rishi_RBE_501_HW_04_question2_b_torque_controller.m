clc
clear all
close all


%% trajectory planner for torque controller 

qi1= 30.6510*(pi/180);
qf1= 98.0312*(pi/180);
qi2= 51.3178*(pi/180);
qf2= 51.3178*(pi/180);

vi1 = 0;
vi2 = 0;

vf1 = 0;
vf2 = 0;

d1 = [qi1,qf1,vi1,vf1,0,5];
[a1] = cubic(d1(1),d1(2),d1(3),d1(4),d1(5),d1(6));
d2 = [qi2,qf2,vi2,vf2,0,5];
[a2] = cubic(d2(1),d2(2),d2(3),d2(4),d2(5),d2(6));
a1;
a2;


sim("Khajuriwala_Rishi_RBE_501_HW_04_question2_b_sim")
function [a] =cubic(qi,qf, vi,vf,ti,tf)
t= tf-ti;
 
A=[1,ti, ti^2,ti^3;
    0,1,2*ti,3*ti^2;
    1,tf, tf^2,tf^3;
    0,1,2*tf,3*tf^2];

B=[qi;vi;qf;vf];
a=A\B;
end



