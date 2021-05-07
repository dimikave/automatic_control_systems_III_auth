%
% KAVELIDIS FRANTZIS DIMITRIOS - AEM 9351 - kavelids@ece.auth.gr - ECE AUTH
% Automatic Control Systems III - Winter Semester Assignment 2020/2021
% 
function phase_portrait(x1,x2,x1dot,x2dot)
    size_x=size(x1);
    for i=1:size_x(2)
        quiver(x1(:,i),x2(:,i),x1dot(:,i),x2dot(:,i))%,'color',[0 0 1]
        hold on;
    end
    hold off;
end
