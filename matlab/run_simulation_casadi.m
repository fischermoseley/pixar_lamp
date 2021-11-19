clear all; close all; clc;
%%

% We can organize our code by filing things in different folders.  These
% folders need to be added to the Matlab path so that it can run the files
% inside them even when they are not the current folder listed at the top
% of the Matlab window.  For more information about the current folder, see
% http://www.mathworks.com/help/matlab/matlab_env/understanding-file-locations-in-matlab.html
% For more information about the Matlab path, see
% http://www.mathworks.com/help/matlab/matlab_env/what-is-the-matlab-search-path.html
setpath                                     % add AutoDerived, Modeling, and Visualization folders to Matlab path

%% Step 0: Setup Casadi
% 1) Download the casadi package for your operating system from https://web.casadi.org/get/
% 2) Unzip the folder, rename it "casadi", and place it in this directory
% 3) Familiarize yourself with casadi by reading (at least) Chapters 1-3, 9
% of https://web.casadi.org/docs/
addpath(genpath('casadi'))

%% Step 1: Setup the Optimization
% Create the optimization object
opti = casadi.Opti();

% Declare Optimization variables
ctrl.tf = opti.variable();    % Duration of control
ctrl.T  = opti.variable(3,1); % Control values

% Time discretization
N.ctrl   = 25; % number of dynamics timesteps where ctrl is applied

% Set parameters         
p  = parameters(); % get robot-specific parameters from file
z0 = [0; pi/6; 0 ;0];

%% Step 3: Build Objective function
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


%% Step 2: Add constraints
% Adding constraints is likewise very simple, just use the
% opti.subject_to() function

% Add lower and upper bounds
opti.subject_to(ctrl.tf >= 0.1);
opti.subject_to(ctrl.tf <= 0.6);
opti.subject_to(ctrl.T >= -2*ones(3,1));
opti.subject_to(ctrl.T <= 2*ones(3,1) );

% Leg angle must stay within bounds
for i = 1:N.ctrl
    opti.subject_to(zout(2,i) <= pi/2);
    opti.subject_to(zout(2,i) >= 0);
end

%% Step 3: Setup the solver options
% (only the ambitious student needs to worry about changing these settings,
% for the most part, you should be able to leave them as is)

p_opts = struct('expand',false);
opti.solver('ipopt',p_opts);

%% Step 4: Provide Initial Guess & Run the optimization
% Setting the initial guess is simple, just use opti.set_initial() for each
% of your optimization variables

% Initial guess
opti.set_initial(ctrl.tf,0.35);
opti.set_initial(ctrl.T,[0. 1.0 1.0]);

% Solve the Optimization
sol = opti.solve();

% Parse solution
tf = sol.value(ctrl.tf)+0.5;          % simulation final time
optimal_ctrl.tf = sol.value(ctrl.tf); % control final time
optimal_ctrl.T  = sol.value(ctrl.T);  % control values

%% Step 5: Simulate and Visualize the Result (same as before mostly)
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
ctrl_pt_t = linspace(0, optimal_ctrl.tf, length(optimal_ctrl.T));

for i=1:length(ctrl_t)
    ctrl_input(i) = BezierCurve(optimal_ctrl.T,ctrl_t(i)/optimal_ctrl.tf);
end
hold on
plot(ctrl_t, ctrl_input);
plot(ctrl_pt_t, optimal_ctrl.T, 'o');
hold off
xlabel('time (s)')
ylabel('torque (Nm)')
title('Control Input Trajectory')
axis([0 0.45 0 2]);

%%
% Run the animation
figure(3)                          % get the coordinates of the points to animate
speed = .25;                                 % set animation speed
cla                                         % clear axes
animate_simple(t,z,p,speed)                 % run animation
