%
% KAVELIDIS FRANTZIS DIMITRIOS - AEM 9351 - kavelids@ece.auth.gr - ECE AUTH
% Automatic Control Systems III - Winter Semester Assignment 2020/2021
% 
%%
%%% -------------------------- Part A ------------------------------------

% In this assignment/Matlab script I will simulate the given system and I
% will plot the Step and Ramp response, as well as the phase portrait, each
% for a number of different initial conditions. This is going to be the
% main file.

% It is suggested to run the code in parts (for every FOR statement)
% using the Evaluate Selection option in order to avoid getting too many
% % figures.
%%
% A)
% III)
% Below I will simulate the system  e_2dot + e_dot + 4e = r_2dot + r_dot    
% using the ode45 function giving an interval of 20 seconds(which is more
% than enough to have a clear view of my system), for a number of initial
% values given by the following matrix:

InitVal = [-2 1.5; -2.5 0.8; 1.5 2; 0.2 1.8; 2.5 -0.8; 2 -2; -0.2 -1.8; -1 -2.5]
% I simulate the system for two different inputs : 
% a) Step Function, b) Ramp Function 

% a) In a different file, we define funcUnit function which creates our
% xdot for Step function input.

% function xdot = funcUnit(t,x)
%     xdot(1) = x(2);
%     xdot(2) = - 4*x(1) - x(2) ;
%     xdot = xdot';
% end
%% For every pair of initial values, we plot the Step Response using the following loop:
for i = 1:8
    figure()
    [t1, state_values] = ode45(@funcUnit,[0,20],[InitVal(i,1) InitVal(i,2)]);
    plot(t1, state_values)
    title("Step Response for initial values x1(0) = " + InitVal(i,1) + " , x2(0) = " + InitVal(i,2));
    legend('x1','x2')
    grid on
    pause;
end  

%% For every pair of initial values, we plot the Phase Portrait using the following loop:
for i = 1:8
    figure()
    [t1, state_values] = ode45(@funcUnit,[0,20],[InitVal(i,1) InitVal(i,2)]);
    x1 = state_values(:,1);
    x2 = state_values(:,2);
    %x1dot = x2;
    %x2dot = -4*x1-x2;
    %phase_portrait(x1,x2,x1dot,x2dot)
    plot(x1,x2)
    hold on
    vectfieldn(@funcUnit,-3.5:.2:3.5,-3.5:.2:3.5) % 
    title("Phase Portrait for initial values x1(0) = " + InitVal(i,1) + " , x2(0) = " + InitVal(i,2));
    grid on
    xlabel('x1')
    ylabel('x2')
    pause;
end

% Note: The function vectfieldn is a function that plots the vector
% field/phasial portrait for a specific system regardless of the initial 
% values, meaning it does not just follow  the trajectory of x, but it
% gives a bigger picture of what happens with the system. For clarity
% purposes, it keeps the same vector length.
% I use it here for the sake of completenss, and it is not a code I made.
% My source for this file(vectfieldn.m) was: 
% http://www-users.math.umd.edu/~petersd/246/matlabode2.html 

%%
% b) In a different file, we define funcRamp function which creates our
% xdot for Step function input.

% function xdot = funcRamp(t,x,V)
%     xdot(1) = x(2);
%     xdot(2) = V - 4*x(1) - x(2) ;
%     xdot = xdot';
% end

%% For every pair of initial values, we plot the Ramp Response using the following loop:
V =1.2
for i = 1:8
    figure()
    [t1, state_values] = ode45(@(t1,state_values) funcRamp(t1,state_values,V),[0,20],[InitVal(i,1) InitVal(i,2)]);
    plot(t1, state_values)
    title("Ramp Response for initial values x1(0) = " + InitVal(i,1) + " , x2(0) = " + InitVal(i,2));
    legend('x1','x2')
    grid on
    pause;
end 

%% For every pair of initial values, we plot the Phase Portrait using the following loop:

for i = 1:8
    figure()
    [t1, state_values] = ode45(@(t1,state_values) funcRamp(t1,state_values,V),[0,100],[InitVal(i,1) InitVal(i,2)]);
    x1 = state_values(:,1);
    x2 = state_values(:,2);
    %x1dot = x2;
    %x2dot = V-4*x1-x2;
    %phase_portrait(x1,x2,x1dot,x2dot)
    plot(x1,x2)
    hold on
    vectfieldn(@(t1,state_values) funcRamp(t1,state_values,V),-4:.2:4,-4:.2:4)
    title("Phase Portrait for initial values x1(0) = " + InitVal(i,1) + " , x2(0) = " + InitVal(i,2));
    xlabel('x1')
    ylabel('x2')
    pause;
end


%% B) Non Linear Part
%Parameters
e0 = 0.2;
a = 0.06;

%% II)a)
%% Our Step Response now:
for i = 1:8
    figure()
    [t1, state_values] = ode45(@(t1,state_values) funcUnitNonLin(t1,state_values,e0,a),[0,20],[InitVal(i,1) InitVal(i,2)]);
    plot(t1, state_values)
    title("Unit Response for initial values x1(0) = " + InitVal(i,1) + " , x2(0) = " + InitVal(i,2));
    legend('x1','x2')
    grid on
    pause;
end 

%% Phase Portrait for step function input / Non linear:
for i = 1:8
    figure()
    [t1, state_values] = ode45(@(t1,state_values) funcUnitNonLin(t1,state_values,e0,a),[0,100],[InitVal(i,1) InitVal(i,2)]);
    % Here I am not going to use phase portrait function, but rather I will simply
    % use plot and vectfield to show how the whole phase plane is in
    % this situation,so we understand how our trajectory is moving. The
    % equation is already solved in the funcUnitNonLin
    x1 = state_values(:,1);
    x2 = state_values(:,2);
    plot(x1,x2)
    hold on
    vectfield(@(t1,state_values) funcUnitNonLin(t1,state_values,e0,a),-3.5:.2:3.5,-3.5:.2:3.5) % 
    % Here it is useful to use vectfield instead of vectfieldn, to check
    % the way the system approaches equilibrium
    title("Phase Portrait for initial values x1(0) = " + InitVal(i,1) + " , x2(0) = " + InitVal(i,2));
    grid on
    xlabel('x1')
    ylabel('x2')
    pause;
end

%% II)b)
% For other scale values and the ones used in the pdf report, we simply change the value of V 

V = 1.2; 
% V = 0.4
% V = 0.04
% Our ramp response now:
for i = 1:8
    figure()
    [t1, state_values] = ode45(@(t1,state_values) funcRampNonLin(t1,state_values,e0,a,V),[0,100],[InitVal(i,1) InitVal(i,2)]);
    plot(t1, state_values)
    title("Ramp Response for initial values x1(0) = " + InitVal(i,1) + " , x2(0) = " + InitVal(i,2));
    legend('x1','x2')
    grid on
    pause;
end 

%% Phase Portrait for ramp function input / Non linear:
for i = 1:8
    figure()
    [t1, state_values] = ode45(@(t1,state_values) funcRampNonLin(t1,state_values,e0,a,V),[0,100],[InitVal(i,1) InitVal(i,2)]);
    % Here I am not going to use phase portrait function, but rather I will simply
    % use plot and vectfield to show how the whole phase plane is in
    % this situation,so we understand how our trajectory is moving. The
    % equation is already solved in the funcRampNonLin
    x1 = state_values(:,1);
    x2 = state_values(:,2);
    plot(x1,x2) 
    hold on
    vectfield(@(t1,state_values) funcRampNonLin(t1,state_values,e0,a,V),-4:.2:4,-4:.2:4) 
    % Here it is useful to use vectfield instead of vectfieldn, to check
    % the way the system approaches equilibrium
    title("Phase Portrait for initial values x1(0) = " + InitVal(i,1) + " , x2(0) = " + InitVal(i,2) + " , V = " + V);
    grid on
    pause;
    xlabel('x1')
    ylabel('x2')
end  

% This part is to test our phase portraits with different input ramps
% V1 = [0.012, 0.024, 0.04, 0.048, 0.4, 0.7, 0.8, 1, 1.2];
% for i = 1:9
%     figure()
%     [t1, state_values] = ode45(@(t1,state_values) funcRampNonLin(t1,state_values,e0,a,V1(i)),[0,100],[0.299999911,0]);
%     % here we test with different initial values
%     x1 = state_values(:,1);
%     x2 = state_values(:,2);
%     plot(x1,x2) 
%     hold on
%     vectfieldn(@(t1,state_values) funcRampNonLin(t1,state_values,e0,a,V1(i)),-4:.2:4,-4:.2:4) 
%     % Here it is useful to use vectfield instead of vectfieldn, to check
%     % the way the system approaches equilibrium
%     title("Phase Portrait for initial values x1(0) = " + 0.3 + " , x2(0) = " + 0 + " , V = " + V(i));
%     grid on
%     pause;
%     xlabel('x1')
%     ylabel('x2')   
%     figure()
%     plot(t1,state_values);
%     pause;
% end  

%%% ------------------------- End of Part A ------------------------------