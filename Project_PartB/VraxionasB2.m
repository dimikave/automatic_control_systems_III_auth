%
% KAVELIDIS FRANTZIS DIMITRIOS - AEM 9351 - kavelids@ece.auth.gr - ECE AUTH
% Automatic Control Systems III - Winter Semester Assignment 2020/2021
% 
%%
%odefun describing our two link robot manipulator
function xdot = VraxionasB2(t,x,epsilon)

    global m1 m2 L1 L2 Lc1 Lc2 Iz1 Iz2 g
    % First we are going to define our matrices/vectors
% ----------------------- 
    q1 = x(1);
    q2 = x(2);
    q1dot = x(3);
    q2dot = x(4);
% ----------------------- 
    H = [(m2*(Lc2^2 + 2*L1*Lc2*cos(q2)+ L1^2) + m1*Lc1^2 + Iz2 + Iz1)...
        (m2*Lc2^2 + L1*Lc2*m2*cos(q2)+Iz2); (m2*Lc2^2 + L1*Lc2*m2*cos(q2) + Iz2)...
        (m2*Lc2^2+Iz2);];
    
    C = [(-m2*L1*Lc2*sin(q2)*q2dot) (-m2*L1*Lc2*sin(q2)*(q2dot+q1dot)); (m2*L1*Lc2*sin(q2)*q1dot) 0];
    
    G = [m2*Lc2*g*cos(q1+q2) + (m2*L1+m1*Lc1)*g*cos(q1); m2*Lc2*g*cos(q1+q2)];
% -----------------------    
    
    t      % we can add this test to check how fast our code works
    u = InputTorqueB2(x(1),x(2),x(3),x(4),t,epsilon);
    xdot(1) = x(3);
    xdot(2) = x(4);
    a = -inv(H)*(C * [x(3) ; x(4)] + G - u);
    xdot(3) = a(1);
    xdot(4) = a(2);
    xdot = xdot';
end