%
% KAVELIDIS FRANTZIS DIMITRIOS - AEM 9351 - kavelids@ece.auth.gr - ECE AUTH
% Automatic Control Systems III - Winter Semester Assignment 2020/2021
% 
%% ------------------------- Part B --------------------------------
% In this assignment, we want to simulate the system that describes the
% movement of a two-link robot arm. The feedback control
% problem for the system is to compute the required actuator input torques to perform
% the desired task: to follow a desired trajectory. For this we are going
% to compare 2 different contollers, one that uses simple feedback
% linearization, and a second one that uses the sliding mode control
% method.

%% B1
%% Clearing everything
clc;
clear;
close all;

%% Assigning the real parameters of the two link robot manipulator
global m1 m2 L1 L2 Lc1 Lc2 Iz1 Iz2 g
global m2hat Lc1hat Lc2hat Iz1hat Iz2hat

    m1 = 6;
    m2 = 4;
    L1 = 0.5;
    L2 = 0.4;
    Lc1 = 0.2;
    Lc2 = 0.4;
    Iz1 = 0.43;
    Iz2 = 0.05;
    g = 9.81;
    
%% Assigning the estimated parameters of the two link robot manipulator
    m2hat = 2;
    Lc1hat = 0.35;
    Lc2hat = 0.1;
    Iz1hat = 0.05;
    Iz2hat = 0.02;

tStart = 0;
% if we want more precise calculations, we are going to put a smaller tStep
% tStep = 0.001;
tStep = 0.1;
tEnd = 10;
tspan = tStart:tStep:tEnd;

%% Solving the differantial equation
[ t , state_values] = ode23s(@Vraxionas, tspan, [-87, 167, 0, 0]);

%% Plot of q1,qd1,e1
figure()
plot(t,state_values(:,1))
hold on
yd1 = [];
for i = 1:size(t)
    yd1p = (-90+50*(1-cos(0.63*t(i))))*(t(i)<=5) + 10*(t(i)>5);
    yd1 = [yd1;yd1p];
end
plot(t,yd1)
error1 = state_values(:,1)-yd1;
plot(t,error1,'--')
grid on
ylabel('Angle of first link')
xlabel('Time')
legend('q_1','q_d_1','e_1')

%% Plot of q2,qd2,e2
figure()
plot(t,state_values(:,2))
hold on
yd2 = [];
for i = 1:size(t)
    yd2p = (170-60*(1-cos(0.63*t(i))))*(t(i)<=5)+50*(t(i)>5);
    yd2 = [yd2;yd2p];
end
plot(t,yd2)
error2 = state_values(:,2)-yd2;
plot(t,error2,'--')
grid on
ylabel('Angle of second link')
xlabel('Time')
legend('q_2','q_d_2','e_2')

%% Plot of q1dot,qd1dot,e1dot
figure()
plot(t,state_values(:,3))
hold on
yd1dot = [];
for i = 1:size(t)
    yd1v = (31.5*sin(0.63*t(i)))*(t(i)<5)+ 0*(t(i)>=5);
    yd1dot = [yd1dot;yd1v];
end
plot(t,yd1dot)
error1v = state_values(:,3)-yd1dot;
plot(t,error1v,'--')
grid on
ylabel('Rotational Velocity of first link')
xlabel('Time')
legend('q_1_d_o_t','q_d_1_d_o_t','e_1_d_o_t')

%% Plot of q2dot,qd2dot,e2dot
figure()
plot(t,state_values(:,4))
hold on
yd2dot = [];
for i = 1:size(t)
    yd2v = (-37.8*sin(0.63*t(i)))*(t(i)<5)+ 0*(t(i)>=5);
    yd2dot = [yd2dot;yd2v];
end
plot(t,yd2dot)
error2v = state_values(:,4)-yd2dot;
plot(t,error2v,'--')
grid on
ylabel('Rotational Velocity of second link')
xlabel('Time')
legend('q_2_d_o_t','q_d_2_d_o_t','e_2_d_o_t')

%% Plot of input u

yd1dotdot = [];
yd2dotdot = [];
for i = 1:size(t)
    yd1a = (19.845*cos(0.63*t(i)))*(t(i)<5)+ 0*(t(i)>=5);
    yd2a = (-23.814*cos(0.63*t(i)))*(t(i)<5)+0*(t(i)>=5);
    yd1dotdot = [yd1dotdot;yd1a];
    yd2dotdot = [yd2dotdot;yd2a];
end

KD = [20 0; 0 20];
KP = [100 0; 0 100];

u = [];
q1 = state_values(:,1);
q2 = state_values(:,2);
q1d = state_values(:,3);
q2d = state_values(:,4);
for i = 1:size(t)
    Hhat = [(m2hat*(Lc2hat^2 + 2*L1*Lc2hat*cos(q2(i))+ L1^2) + m1*Lc1hat^2 ...
        + Iz2hat + Iz1hat) (m2hat*Lc2hat^2 + L1*Lc2hat*m2hat*cos(q2(i))+Iz2hat);...
        (m2hat*Lc2hat^2 + L1*Lc2hat*m2hat*cos(q2(i)) + Iz2hat) (m2hat*Lc2hat^2+Iz2hat);];
    
    Chat = [(-m2hat*L1*Lc2hat*sin(q2(i))*q2d(i)) (-m2hat*L1*Lc2hat*sin(q2(i))*(q2d(i)+q1d(i)));...
        (m2hat*L1*Lc2hat*sin(q2(i))*q1d(i)) 0];
    
    Ghat = [m2hat*Lc2hat*g*cos(q1(i)+q2(i)) + (m2hat*L1+m1*Lc1hat)*g*cos(q1(i));...
        m2hat*Lc2hat*g*cos(q1(i)+q2(i))];
    uprin = Hhat*([yd1dotdot(i);yd2dotdot(i)]+KD*([yd1dot(i);yd2dot(i)]-[q1d(1);q2d(i)])+KP*([yd1(i);yd2(i)]-[q1(i);q2(i)]))+Chat*[q1d(1);q2d(i)]+Ghat;
    u = [u;uprin(1) uprin(2)];
end

figure()
plot(t,u(:,1))
ylabel('Torque of joint 1')
xlabel('Time')
grid on
figure()
plot(t,u(:,2))
ylabel('Torque of joint 2')
xlabel('Time')
grid on

%% Pausing between simulations
pause;

%% B2: Sliding mode control
%% Solving differential equations
epsilon = 0.0001;
[tB,x] = ode23s(@(t,state_values) VraxionasB2(t , state_values,epsilon), tspan, [-87, 167, 0, 0]);

%% Plot of q1,qd1,e1
figure()
plot(tB,x(:,1))
hold on
plot(tB,yd1)
errorB1 = x(:,1)-yd1;
plot(tB,errorB1,'--')
grid on
ylabel('Angle of first link')
xlabel('Time')
legend('q_1','q_d_1','e_1')

%% Plot of q2,qd2,e2
figure()
plot(tB,x(:,2))
hold on
plot(tB,yd2)
errorB2 = x(:,2)-yd2;
plot(tB,errorB2,'--')
grid on
ylabel('Angle of second link')
xlabel('Time')
legend('q_2','q_d_2','e_2')

%% Plot of q1dot,qd1dot,e1dot
figure()
plot(tB,x(:,3))
hold on
plot(tB,yd1dot)
errorB1v = x(:,3)-yd1dot;
plot(tB,errorB1v,'--')
grid on
ylabel('Rotational Velocity of first link')
xlabel('Time')
legend('q_1_d_o_t','q_d_1_d_o_t','e_1_d_o_t')

%% Plot of q2dot,qd2dot,e2dot
figure()
plot(tB,x(:,4))
hold on
plot(tB,yd2dot)
errorB2v = x(:,4)-yd2dot;
plot(tB,errorB2v,'--')
grid on
ylabel('Rotational Velocity of second link')
xlabel('Time')
legend('q_2_d_o_t','q_d_2_d_o_t','e_2_d_o_t')

%% Plot of input u
lamda = 10
L = [lamda 0; 0 lamda];
uB = [];
q1B = x(:,1);
q2B = x(:,2);
q1dB = x(:,3);
q2dB = x(:,4);
for i = 1:size(tB)
    HhatB = [(m2hat*(Lc2hat^2 + 2*L1*Lc2hat*cos(q2B(i))+ L1^2) + m1*Lc1hat^2 ...
        + Iz2hat + Iz1hat) (m2hat*Lc2hat^2 + L1*Lc2hat*m2hat*cos(q2B(i))+Iz2hat);...
        (m2hat*Lc2hat^2 + L1*Lc2hat*m2hat*cos(q2B(i)) + Iz2hat) (m2hat*Lc2hat^2+Iz2hat);];
    
    ChatB = [(-m2hat*L1*Lc2hat*sin(q2B(i))*q2dB(i)) (-m2hat*L1*Lc2hat*sin(q2B(i))*(q2dB(i)+q1dB(i)));...
        (m2hat*L1*Lc2hat*sin(q2B(i))*q1dB(i)) 0];
    
    GhatB = [m2hat*Lc2hat*g*cos(q1B(i)+q2B(i)) + (m2hat*L1+m1*Lc1hat)*g*cos(q1B(i));...
        m2hat*Lc2hat*g*cos(q1B(i)+q2B(i))];
    er = [errorB1(i);errorB2(i)];
    erdot = [errorB1v(i);errorB2v(i)];
    s = erdot + L*er;
    qrdot = [yd1dot(i);yd2dot(i)]-L*er;
    qrdd = [yd1dotdot(i);yd2dotdot(i)]-L*erdot;
    Y1 = [g*cos(q1B(i))*L1 g*cos(q1B(i))*m1 g*cos(q1B(i)+q2B(i))-L1*qrdot(1)*q2dB(i)*sin(q2B(i))-L1*qrdot(2)*sin(q2B(i))*(q1dB(i)+q2dB(i)); 0 0 L1*qrdot(1)*q1dB(i)*sin(q2B(i))+g*cos(q1B(i)+q2B(i))];
    theta1 = [3;0.25;2.05];
    Y2 = [L1^2*qrdd(1) L1*cos(q2B(i))*(2*qrdd(1)+qrdd(2)) (qrdd(1)+qrdd(2)) m1*qrdd(1) (qrdd(1)+qrdd(2)) qrdd(1);...
        0 L1*cos(q2B(i))*qrdd(1) (qrdd(1)+qrdd(2)) 0 (qrdd(1)+qrdd(2)) 0];
    theta2 = [3;2.05;0.9925;0.1125;0.13;0.45];
    h = [0.1; 0.2];
    K = abs(Y1)*theta1+abs(Y2)*theta2+h;
    a = SmoothSign(s,epsilon);
    uprinB = HhatB*qrdd+ChatB*qrdot+GhatB-K.*a;
    uB = [uB;uprinB(1) uprinB(2)];
end

figure()
plot(tB,uB(:,1))
ylabel('Torque of joint 1')
xlabel('Time')
grid on
figure()
plot(tB,uB(:,2))
ylabel('Torque of joint 2')
xlabel('Time')
grid on

%% Plot of sliding surface s
s1 = errorB1v - lamda*errorB1;
s2 = errorB2v - lamda*errorB2;
ss = [s1 s2];

% figure()
% plot(tB,s1)
% grid on
% xlabel('Time')
% ylabel('s1')
% 
% figure()
% plot(tB,s2)
% grid on
% xlabel('Time')
% ylabel('s2')

figure()
plot(tB,ss)
legend('s_1','s_2')
xlabel('Time')
ylabel('s_1, s_2')

figure()
plot(errorB1,errorB1v);
title('Sliding Surface, lamda = 10')
ylabel('e_1_d_o_t')
xlabel('e_1')

figure
plot(errorB2,errorB2v);
grid on
ylabel('e_2_d_o_t')
xlabel('e_2')


figure()
plot(errorB1,errorB1v);
title('Sliding Surface, lamda = 10')
grid on
ylabel('e_d_o_t')
xlabel('e')
hold on
plot(errorB2,errorB2v);
hold on
syms surf
y = -10*surf;
fplot(y,'-.')
legend ('e_1','e_2','s = 0')

figure()
plot(s1,s2)
title('Sliding Surface, lamda = 10')
ylabel('s_2')
xlabel('s_1')

pause;
%% Epsilon = 0.1 test
epsilon =0.1;
% epsilon =100;
[tst,Test] = ode23s(@(t,state_values) VraxionasB2(t , state_values,epsilon), tspan, [-87, 167, 0, 0]);

figure()
plot(tst,Test(:,1))
hold on
ylabel('q_1,q_d_1')
xlabel('Time')
plot(tst,yd1)
legend('q_1','q_d_1')
grid on


figure()
plot(tst,Test(:,2))
hold on
ylabel('q_2,q_d_2')
xlabel('Time')
plot(tst,yd2)
legend('q_2','q_d_2')
grid on

lamda = 10;
L = [lamda 0; 0 lamda];
uT = [];
q1B = Test(:,1);
q2B = Test(:,2);
q1dB = Test(:,3);
q2dB = Test(:,4);
errorB1v = Test(:,3)-yd1dot;
errorB2v = Test(:,4)-yd2dot;
errorB1 = q1B - yd1;
errorB2 = q2B - yd2;
for i = 1:size(tst)
    HhatB = [(m2hat*(Lc2hat^2 + 2*L1*Lc2hat*cos(q2B(i))+ L1^2) + m1*Lc1hat^2 ...
        + Iz2hat + Iz1hat) (m2hat*Lc2hat^2 + L1*Lc2hat*m2hat*cos(q2B(i))+Iz2hat);...
        (m2hat*Lc2hat^2 + L1*Lc2hat*m2hat*cos(q2B(i)) + Iz2hat) (m2hat*Lc2hat^2+Iz2hat);];
    
    ChatB = [(-m2hat*L1*Lc2hat*sin(q2B(i))*q2dB(i)) (-m2hat*L1*Lc2hat*sin(q2B(i))*(q2dB(i)+q1dB(i)));...
        (m2hat*L1*Lc2hat*sin(q2B(i))*q1dB(i)) 0];
    
    GhatB = [m2hat*Lc2hat*g*cos(q1B(i)+q2B(i)) + (m2hat*L1+m1*Lc1hat)*g*cos(q1B(i));...
        m2hat*Lc2hat*g*cos(q1B(i)+q2B(i))];
    er = [errorB1(i);errorB2(i)];
    erdot = [errorB1v(i);errorB2v(i)];
    s = erdot + L*er;
    qrdot = [yd1dot(i);yd2dot(i)]-L*er;
    qrdd = [yd1dotdot(i);yd2dotdot(i)]-L*erdot;
    Y1 = [g*cos(q1B(i))*L1 g*cos(q1B(i))*m1 g*cos(q1B(i)+q2B(i))-L1*qrdot(1)*q2dB(i)*sin(q2B(i))-L1*qrdot(2)*sin(q2B(i))*(q1dB(i)+q2dB(i)); 0 0 L1*qrdot(1)*q1dB(i)*sin(q2B(i))+g*cos(q1B(i)+q2B(i))];
    theta1 = [3;0.25;2.05];
    Y2 = [L1^2*qrdd(1) L1*cos(q2B(i))*(2*qrdd(1)+qrdd(2)) (qrdd(1)+qrdd(2)) m1*qrdd(1) (qrdd(1)+qrdd(2)) qrdd(1);...
        0 L1*cos(q2B(i))*qrdd(1) (qrdd(1)+qrdd(2)) 0 (qrdd(1)+qrdd(2)) 0];
    theta2 = [3;2.05;0.9925;0.1125;0.13;0.45];
    h = [0.1; 0.2];
    K = abs(Y1)*theta1+abs(Y2)*theta2+h;
    a = SmoothSign(s,epsilon);
    uprinB = HhatB*qrdd+ChatB*qrdot+GhatB-K.*a;
    uT = [uT;uprinB(1) uprinB(2)];
end

s1 = errorB1v - lamda*errorB1;
s2 = errorB2v - lamda*errorB2;
ss = [s1 s2];

% figure()
% plot(tB,s1)
% grid on
% xlabel('Time')
% ylabel('s1')
% 
% figure()
% plot(tB,s2)
% grid on
% xlabel('Time')
% ylabel('s2')

figure()
plot(tst,ss)
legend('s_1','s_2')
xlabel('Time')
ylabel('s_1, s_2')

figure()
plot(s1,s2)
xlabel('Time')



figure()
plot(tst,uT(:,1))
ylabel('Torque of joint 1')
xlabel('Time')
grid on
figure()
plot(tst,uT(:,2))
ylabel('Torque of joint 2')
xlabel('Time')
grid on