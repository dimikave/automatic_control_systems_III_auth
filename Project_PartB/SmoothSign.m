%
% KAVELIDIS FRANTZIS DIMITRIOS - AEM 9351 - kavelids@ece.auth.gr - ECE AUTH
% Automatic Control Systems III - Winter Semester Assignment 2020/2021
% 
%%
function a = SmoothSign(s,epsilon)
    if abs(s(1))>=epsilon
        a1 = s(1)/abs(s(1));
    else
        a1 = s(1)/epsilon;
    end
    if abs(s(2))>=epsilon
        a2 = s(2)/abs(s(2));
    else
        a2 = s(2)/epsilon;
    end
    a = [a1;a2];
end