% Optimize the jump height of a two-mass system, and plot results.

% Housekeeping
clear all; close all; clc;
addpath([pwd '/casadi'])
addpath(genpath('casadi'))

% Parameters
N_ctrl  = 100; % number of dynamics timesteps where ctrl is applied
params.m1 = 1;
params.m2 = 1;
params.kl = 0; %200;
params.dl = 0; %0.1;
params.g = 9.81;
params.xd = 0.5;
params.kg = 12245;
params.dg = 10;
force_limit = 100;
max_travel = 10;

% Initial state
z0 = [1; 0.5; 0 ;0];

% Declare decision variables
opti = casadi.Opti();
ctrl.tf = opti.variable();    % Duration of control
ctrl.u  = opti.variable(1, N_ctrl); % Control values

% Cost function
[tout,q] = stance_simulation_casadi(z0,ctrl,params,N_ctrl);
com = (q(1, N_ctrl)*params.m1 + q(2, N_ctrl)*params.m2)/(params.m1+params.m2);
%com = q(4,N_ctrl);
opti.minimize(-com);

% Constrain actuator force
opti.subject_to(ctrl.tf >= 0.1);
opti.subject_to(ctrl.tf <= 0.6);
opti.subject_to(ctrl.u >= -force_limit*ones(1, N_ctrl));
opti.subject_to(ctrl.u <=  force_limit*ones(1, N_ctrl));

% Can't extend the spring too much
for i = 1:N_ctrl
    opti.subject_to(q(1,i) - q(2,i) <= max_travel);
    opti.subject_to(q(1,i) - q(2,i) >= 0);
end

p_opts = struct('expand',false);
opti.solver('ipopt',p_opts);

% Set initial guess
opti.set_initial(ctrl.tf,0.35);
opti.set_initial(ctrl.u, ones(1,N_ctrl));

% Solve the Optimization
sol = opti.solve();

% Extract solution
tf = sol.value(ctrl.tf)+0.5;          % simulation final time
optimal_ctrl.tf = sol.value(ctrl.tf); % control final time
optimal_ctrl.u  = sol.value(ctrl.u);  % control values

% Plotting
[t, q] = hybrid_simulation_sol(z0,optimal_ctrl,params,N_ctrl);

% Display results
figure(1)
subplot(1,2,1)
plot(t,q(1,:))
hold on
plot(t,q(2,:))
plot(t, (params.m1*q(1,:)+params.m2*q(2,:))/(params.m1+params.m2))
xlabel('Time (s)')
ylabel('Height (m)')
title('Leg Trajectory')
legend('Hip','Foot', 'CoM')
hold off 

subplot(1,2,2)
plot(t, q(3,:))
hold on
plot(t, q(4,:))
plot(t, (params.m1*q(3,:)+params.m2*q(4,:))/(params.m1+params.m2))
xlabel('time (s)')
ylabel('Velocity (m/s)')
title('Leg Trajectory')
legend('Hip','Foot')
hold off 

figure(2)
subplot(1,2,1)
plot(t, optimal_ctrl.u);
xlabel('Time (s)')
ylabel('Actuator Force (N)')

subplot(1,2,2)
plot(t, q(1,:) - q(2,:));
xlabel('Time (s)')
ylabel('Leg Separation (m)')

% Animation
%m = gen_movie(t, q);
%movie(m)