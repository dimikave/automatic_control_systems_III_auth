-----------------------
Semester Project Part A
-----------------------

Project Description:

- Running a simulation of a closed loop system (ode45)
  i)with and ii)without a discontinuous nonlinearity
  using a) step function b) ramp function
  as input
- Conclude about system stability

Files:

- funcRamp.m                      : odefun for Ramp function input on the linear system
- funcRampNonLin.m                : odefun for Ramp function input on the non linear system
- funcUnit.m                      : odefun for Step function input on the linear system
- funcUnitNonLin.m                : odefun for Step function input on the non linear system
- phase_portrait.m                : function for phase portrait plot
- vectfield.m                     : vector field for system of 2 first order ODEs
- vectfieldn.m                    : vector field for system of 2 first order ODEs with arrows normalized to the same length
- MainA.m                         : Main MATLAB file running the simulation and generating diagrams
- PartA.pdf                       : Report for the Project  (in Greek)

Kavelidis Frantzis Dimitris								20/1/2021
