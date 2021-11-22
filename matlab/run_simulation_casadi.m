clear all; close all; clc;
setpath % add AutoDerived, Modeling, and Visualization folders to Matlab path
addpath(genpath('casadi'))

%% Step 1: Setup the Optimization
% Create the optimization object
opti = casadi.Opti();

% Declare Optimization variables
ctrl.tf = opti.variable();    % Duration of control
ctrl.T1  = opti.variable(5,1); % Control values for elbow torques
ctrl.T2  = opti.variable(5,1); % Control values for shoulder torques

% Time discretization
N.ctrl   = 50; % number of dynamics timesteps where ctrl is applied

% Set parameters         
p  = parameters(); % get robot-specific parameters from file
z0 = [0; pi/2; 0; 0; 0; 0];

%% Step 2: Build Objective function
% It is very easy to add an objective function to a casadi optimization.
% Simply use the opti.minimize() function
% Note: Using these function overwrites the last time it was used, so only use it once 
%
% Also note that we are only optimizing over the stance portion (not
% flight), so we set up an equivalent cost function to maximum COM height:
% maximum COM velocity at takeoff
[tout,zout,uout] = stance_simulation_casadi(z0,ctrl,p,tf,N);

% Maximize the COM velocity at takeoff
COM = COM_jumping_leg(zout,p); 
opti.minimize(-COM(4,N.ctrl));


%% Step 3: Add constraints
% Bounding box on flight time/joint torques
opti.subject_to(ctrl.tf >= 0.1);
opti.subject_to(ctrl.tf <= 0.6);
opti.subject_to(ctrl.T1 >= -2*ones(5,1));
opti.subject_to(ctrl.T1 <= 2*ones(5,1) );
opti.subject_to(ctrl.T2 >= -2*ones(5,1));
opti.subject_to(ctrl.T2 <= 2*ones(5,1) );

% Bounding box on leg angle
for i = 1:N.ctrl
    % shoulder
    opti.subject_to(zout(2,i) <= pi/2);
    opti.subject_to(zout(2,i) >= 0);
    % elbow
    opti.subject_to(zout(3,i) <= pi/2);
    opti.subject_to(zout(3,i) >= -pi/2);
end

%% Step 3: Setup the solver options
% (only the ambitious student needs to worry about changing these settings,
% for the most part, you should be able to leave them as is)

p_opts = struct('expand',false);
opti.solver('ipopt',p_opts);

%% Step 4: Provide Initial Guess & Run the optimization
% Initial guess
opti.set_initial(ctrl.tf,0.35);
opti.set_initial(ctrl.T1,[0. 1.0 1.0 1.0 1.0]);
opti.set_initial(ctrl.T2,[0. 1.0 1.0 1.0 1.0]);

sol = opti.solve();

% Parse solution
tf = sol.value(ctrl.tf)+0.5;          % simulation final time
optimal_ctrl.tf = sol.value(ctrl.tf); % control final time
optimal_ctrl.T1  = sol.value(ctrl.T1);  % control values
optimal_ctrl.T2  = sol.value(ctrl.T2);

%% Step 5: Simulate and Visualize the Result
[t z u indices] = hybrid_simulation_sol(z0,optimal_ctrl,p,[0 tf]); % run simulation

figure(1)
COM = COM_jumping_leg(z,p);
subplot(1,2,1)
plot(t,COM(2,:))
xlabel('time (s)')
ylabel('CoM Height (m)')
title('Center of Mass Trajectory')
subplot(1,2,2)
plot(t,COM(4,:))
xlabel('time (s)')
ylabel('CoM Vertical Velocity (m/s)')
title('Center of Mass Vel. Trajectory')

figure(2)  % control input profile
ctrl_t = linspace(0, optimal_ctrl.tf, 50);
ctrl_pt_t = linspace(0, optimal_ctrl.tf, length(optimal_ctrl.T1));

for i=1:length(ctrl_t)
    ctrl1_input(i) = BezierCurve(optimal_ctrl.T1,ctrl_t(i)/optimal_ctrl.tf);
    ctrl2_input(i) = BezierCurve(optimal_ctrl.T2,ctrl_t(i)/optimal_ctrl.tf);
end
hold on
plot(ctrl_t, ctrl1_input);
plot(ctrl_t, ctrl2_input);
plot(ctrl_pt_t, optimal_ctrl.T1, 'o');
plot(ctrl_pt_t, optimal_ctrl.T2, 'x');
hold off
xlabel('time (s)')
ylabel('torque (Nm)')
title('Control Input Trajectory')
axis([0 0.45 -3 3]);

%% Step 6: Run the animation
figure(3)                   % get the coordinates of the points to animate
speed = .1;                 % set animation speed
cla                         % clear axes
animate_simple(t,z,p,speed) % run animation
