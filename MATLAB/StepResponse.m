clc
clear
%declare variables
g = 9.8; % gravity
rR = 0.06; %Radius of Reaction Wheel
l = 0.133; %Lenght of rod
mM = 0.11; %mass of motor  0.11
mP = 0.027; % mass of pendulum rod in kg.  0.027
mR = 0.06; %mass of Reaction wheel   0.06
M = 0.5*mP + mR; %used for convenience
IR = (0.5)*mR*(rR^2); %moment of intertia of reaction wheel as it spins
Im = mM*(l^2); % moment of inertia of rod due to motor as its attached to the end of the rod
IRW = mR*(l^2); % the reaction wheel is also setting on top of the rod so its mass must be accounted for
Ip = (1/3)*mP*(l^2); %moment of inertia of rod due to its own mass
I = Ip + Im + IRW; %Total moment of inertia of rod due to the 3 masses (mM, mP, mR)
kt = 0.05; %Torque constant. I guessed it. The value can be anywhere from 0.05 to 0.2
k = kt; %also guessed
R = 0.1; %Internal resistance of motor anywhere from 0.1 to 1 ohm
induct = 0.00001; %inductance of motor could be anywhere form 0.00001 to 0.00005

%create state space matracies
A = [0 1 0 0; ((M*g)/l) 0 0 (-kt/I); 0 0 0 (kt/IR); 0 (k/induct) (-k/induct) (-R/induct)];
B = [0; 0; 0; (1/induct)];
C = [ 1 0 0 0; 0 0 0 0; 0 0 0 0; 0 0 0 0];
D = 0;
% create a steate space model object called sys
sys = ss(A,B,C,D);

t = 0:0.2:5; % Time vector for simulation
u = 1*ones(size(t)); % Input signal (step input)

% Initial condition for the pendulum angle (theta) in radians
initial_condition = [0; 0; 0; 0]; % 0 rads

% Simulate the step response with the given initial condition
[y, t] = lsim(sys, u, t, initial_condition);

[y,t] = step(sys,t);
var = step(sys,t);
% get step response and save it into y and t
%[y, t] = lsim(sys, u, t);


%graphing. you can copy this into the command window if you want
% Plot the step response
plot(t, y);
xlabel('Time');
ylabel('Output');
title('Step Response');
grid on;