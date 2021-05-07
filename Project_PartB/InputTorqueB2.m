%
% KAVELIDIS FRANTZIS DIMITRIOS - AEM 9351 - kavelids@ece.auth.gr - ECE AUTH
% Automatic Control Systems III - Winter Semester Assignment 2020/2021
% 
%%
function u = InputTorqueB2(q1,q2,q1d,q2d,t,epsilon)

    % Here we are using the estimated values so the
    global m2hat Lc1hat Lc2hat Iz1hat Iz2hat
    global m1 L1 g L2
    
    Hhat = [(m2hat*(Lc2hat^2 + 2*L1*Lc2hat*cos(q2)+ L1^2) + m1*Lc1hat^2 ...
        + Iz2hat + Iz1hat) (m2hat*Lc2hat^2 + L1*Lc2hat*m2hat*cos(q2)+Iz2hat);...
        (m2hat*Lc2hat^2 + L1*Lc2hat*m2hat*cos(q2) + Iz2hat) (m2hat*Lc2hat^2+Iz2hat);];
    
    Chat = [(-m2hat*L1*Lc2hat*sin(q2)*q2d) (-m2hat*L1*Lc2hat*sin(q2)*(q2d+q1d));...
        (m2hat*L1*Lc2hat*sin(q2)*q1d) 0];
    
    Ghat = [m2hat*Lc2hat*g*cos(q1+q2) + (m2hat*L1+m1*Lc1hat)*g*cos(q1);...
        m2hat*Lc2hat*g*cos(q1+q2)];
    
    L = [10 0; 0 10];
    qd = [q1d; q2d];
    yd1p = (-90+50*(1-cos(0.63*t)))*(t<=5) + 10*(t>5);
    yd1v = (31.5*sin(0.63*t))*(t<5)+ 0*(t>=5);
    yd1a = (19.845*cos(0.63*t))*(t<5)+ 0*(t>=5);
    yd2p = (170-60*(1-cos(0.63*t)))*(t<=5)+50*(t>5);
    yd2v = (-37.8*sin(0.63*t))*(t<5)+ 0*(t>=5);
    yd2a = (-23.814*cos(0.63*t))*(t<5)+0*(t>=5);
    ydp = [yd1p; yd2p];
    ydv = [yd1v; yd2v];
    yda = [yd1a; yd2a];
    xv = [q1d; q2d];
    xp = [q1; q2];
    e = xp-ydp;
    edot = xv-ydv;
    s = edot + L*e;
    qrdot = ydv - L*e;
    qrdd = yda - L*edot;
    Y1 = [g*cos(q1)*L1 g*cos(q1)*m1 g*cos(q1+q2)-L1*qrdot(1)*q2d*sin(q2)-L1*qrdot(2)*sin(q2)*(q1d+q2d); 0 0 L1*qrdot(1)*q1d*sin(q2)+g*cos(q1+q2)];
    theta1 = [3;0.25;2.05];
    Y2 = [L1^2*qrdd(1) L1*cos(q2)*(2*qrdd(1)+qrdd(2)) (qrdd(1)+qrdd(2)) m1*qrdd(1) (qrdd(1)+qrdd(2)) qrdd(1);...
        0 L1*cos(q2)*qrdd(1) (qrdd(1)+qrdd(2)) 0 (qrdd(1)+qrdd(2)) 0];
    theta2 = [3;2.05;0.9925;0.1125;0.13;0.45];
    h = [0.1; 0.2];
    K = abs(Y1)*theta1+abs(Y2)*theta2+h;
    a = SmoothSign(s,epsilon);
    u = Hhat*qrdd+Chat*qrdot+Ghat-K.*a;
end