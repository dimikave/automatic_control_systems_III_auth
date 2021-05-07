%
% KAVELIDIS FRANTZIS DIMITRIOS - AEM 9351 - kavelids@ece.auth.gr - ECE AUTH
% Automatic Control Systems III - Winter Semester Assignment 2020/2021
% 
%%
function u = InputTorque(q1,q2,q1d,q2d,t)

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
    
    qd = [q1d; q2d];
    yd1p = ((-90*pi/180)+(50*pi/180)*(1-cos(0.63*t)))*(t<=5) + (10*pi/180)*(t>5);
    yd1v = ((31.5*pi/180)*sin(0.63*t))*(t<5)+ 0*(t>=5);
    yd1a = ((19.845*pi/180)*cos(0.63*t))*(t<5)+ 0*(t>=5);
    yd2p = ((170*pi/180)-(60*pi/180)*(1-cos(0.63*t)))*(t<=5)+(50*pi/180)*(t>5);
    yd2v = ((-37.8*pi/180)*sin(0.63*t))*(t<5)+ 0*(t>=5);
    yd2a = ((-23.814*pi/180)*cos(0.63*t))*(t<5)+0*(t>=5);
    ydp = [yd1p; yd2p];
    ydv = [yd1v; yd2v];
    yda = [yd1a; yd2a];
    KD = [20 0; 0 20];
    KP = [100 0; 0 100];
    xv = [q1d; q2d];
    xp = [q1; q2];
    u = Hhat*(yda+KD*(ydv-xv)+KP*(ydp-xp))+Chat*qd+Ghat;
end