%
% KAVELIDIS FRANTZIS DIMITRIOS - AEM 9351 - kavelids@ece.auth.gr - ECE AUTH
% Automatic Control Systems III - Winter Semester Assignment 2020/2021
% 
function xdot = funcRamp(t,x,V)
    xdot(1) = x(2);
    xdot(2) = V - 4*x(1) - x(2) ;
    xdot = xdot';
end