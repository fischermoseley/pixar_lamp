clear all; close all; clc;
setpath
addpath(genpath('casadi'))

%% Set up optimization
opti = casadi.Opti();

% Declare Optimization variables
ctrl.tf = opti.variable();    % Duration of control
ctrl.T  = opti.variable(3,1); % Control values

% Time discretization
N.ctrl   = 25; % number of dynamics timesteps where ctrl is applied

% Set parameters         
p  = parameters(); % get robot-specific parameters from file
z0 = [0; pi/6; 0 ;0];

k_ground = 0;
d_ground = 0.2;
[tout,zout,uout] = stance_simulation_casadi(z0,ctrl,p,tf,N, k_ground, d_ground);

% Maximize the COM velocity at takeoff
COM = COM_jumping_leg(zout,p); 
opti.minimize(-COM(4,N.ctrl));

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

p_opts = struct('expand',false);
opti.solver('ipopt',p_opts);

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
[t z u indices] = hybrid_simulation_sol(z0,optimal_ctrl,p,[0 tf], k_ground, d_ground); % run simulation

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
speed = .05;                                 % set animation speed
cla                                         % clear axes
animate_simple(t,z,p,speed)                 % run animation
