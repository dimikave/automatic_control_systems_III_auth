%
% KAVELIDIS FRANTZIS DIMITRIOS - AEM 9351 - kavelids@ece.auth.gr - ECE AUTH
% Automatic Control Systems III - Winter Semester Assignment 2020/2021
% 
function xdot = funcRampNonLin(t,x,e0,a,V)
    xdot(1) = x(2);
    if (x(1)>= -e0) && (x(1) <= e0)
        xdot(2) = V - 4*a*x(1) - x(2);
    else
        xdot(2) = V - 4*x(1) - x(2);
    end
    xdot = xdot';
end