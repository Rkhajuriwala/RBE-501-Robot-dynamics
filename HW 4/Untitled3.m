syms t a0 a1 a2 a3 a4 a5
syms t0 pos0 vel0 accel0 tf posf velf accelf % Initial and Final
% position, velocity, and acceleration
q = a5*t^5 + a4*t^4 + a3*t^3 + a2*t^2 + a1*t^1 + a0
dq = diff(q,t)
ddq = diff(dq, t)
% Prepare variables
x_start = -300; y_start = 450; % in mm
x_end = 300; y_end = 450;
times = linspace(0,5,100); % in seconds
link = 300; % in mm
mass = 0.05; % in Kg
% Get the inverse kinematics for the two link arm
theta2_start = acos((x_start^2 + y_start^2 - 2*link^2) / (2*link^2))
theta1_start = atan2(y_start, x_start) - atan2((link*sin(theta2_start)), (link+link*cos(theta2_start)))
theta2_end = acos((x_end^2 + y_end^2 - 2*link^2) / (2*link^2))
theta1_end = atan2(y_end, x_end) - atan2((link*sin(theta2_end)), (link+link*cos(theta2_end)))
% For the first joint
a = [1 t0 t0^2 t0^3 t0^4 t0^5;
0 1 2*t0 3*t0^2 4*t0^3 5*t0^4;
0 0 2 6*t0 12*t0^2 20*t0^3;
1 tf tf^2 tf^3 tf^4 tf^5;
0 1 2*tf 3*tf^2 4*tf^3 5*tf^4;
0 0 2 6*tf 12*tf^2 20*tf^3];
b = [a0; a1; a2; a3; a4; a5];
c = [pos0; vel0; accel0; posf; velf; accelf];
C = subs(c,[pos0 vel0 accel0 posf velf accelf], ...
 [theta1_start 0 0 theta1_end 0 0]);
A = subs(a, [t0, tf], [0, 5]);
B = A\C;
calc_pos_q1 = subs(q, [a0 a1 a2 a3 a4 a5], [B(1) B(2) B(3) B(4) B(5) B(6)]);
calc_vel_q1 = subs(dq, [a0 a1 a2 a3 a4 a5], [B(1) B(2) B(3) B(4) B(5) B(6)]);
calc_acc_q1 = subs(ddq,[a0 a1 a2 a3 a4 a5], [B(1) B(2) B(3) B(4) B(5) B(6)]);
q1_temp = subs(calc_pos_q1,t,times);
dq1_temp = subs(calc_vel_q1,t,times);
ddq1_temp = subs(calc_acc_q1,t,times);
% And now for the second joint
C = subs(c,[pos0 vel0 accel0 posf velf accelf], ...
 [theta2_start 0 0 theta2_end 0 0]);
A = subs(a, [t0, tf], [0, 5]);
B = A\C;
calc_pos_q2 = subs(q, [a0 a1 a2 a3 a4 a5], [B(1) B(2) B(3) B(4) B(5) B(6)]);
calc_vel_q2 = subs(dq, [a0 a1 a2 a3 a4 a5], [B(1) B(2) B(3) B(4) B(5) B(6)]);
calc_acc_q2 = subs(ddq,[a0 a1 a2 a3 a4 a5], [B(1) B(2) B(3) B(4) B(5) B(6)]);
q2_temp = subs(calc_pos_q2,t,times);
dq2_temp = subs(calc_vel_q2,t,times);
ddq2_temp = subs(calc_acc_q2,t,times);
% Now Plot the results
subplot(1,2,1)
plot(times,q1_temp)
xlabel('t(s)');
ylabel('Joint q1 position')
grid on
subplot(1,2,2)
plot(times,q2_temp)
xlabel('t(s)');
ylabel('Joint q2 position')
grid on
figure
subplot(1,2,1)
plot(times,dq1_temp)
xlabel('t(s)');
ylabel('Joint q1 velocity')
grid on
subplot(1,2,2)
plot(times,dq2_temp)
xlabel('t(s)');
ylabel('Joint q2 velocity')
grid on
figure
subplot(1,2,1)
plot(times,ddq1_temp)
xlabel('t(s)');
ylabel('Joint q1 acceleration')
grid on
subplot(1,2,2)
plot(times,ddq2_temp)
xlabel('t(s)');
ylabel('Joint q2 acceleration')
grid on
% plotting the final trajectory of the arm.
l1 = 300;
l2 = 300;
figure
for i=1:numel(times)
x_1 = l1*cos(q1_temp(i));
y_1 = l1*sin(q1_temp(i));
x_2 = l1*cos(q1_temp(i)) + l2*cos(q1_temp(i) + q2_temp(i));
y_2 = l1*sin(q1_temp(i)) + l2*sin(q1_temp(i) + q2_temp(i));
plot([0 x_1 x_2], [0 y_1 y_2], '-gs',...
 'LineWidth',2,...
 'MarkerSize',10,...
 'MarkerEdgeColor','b',...
 'MarkerFaceColor',[0.5,0.5,0.5]);
axis([-500 500 -50 600])
grid on
pause(0.01);
end