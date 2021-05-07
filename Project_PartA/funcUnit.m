%
% KAVELIDIS FRANTZIS DIMITRIOS - AEM 9351 - kavelids@ece.auth.gr - ECE AUTH
% Automatic Control Systems III - Winter Semester Assignment 2020/2021
% 
function xdot = funcUnit(t,x)
    xdot(1) = x(2);
    xdot(2) = - 4*x(1) - x(2) ;
    xdot = xdot';
end