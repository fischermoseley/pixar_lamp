clear all; close all; clc;
setpath
addpath(genpath('casadi'))

%% Step 1: Setup the Optimization
opti = casadi.Opti();

% Time discretization
N_ctrl  = 25; % number of dynamics timesteps where ctrl is applied

% Declare Optimization variables
ctrl.tf = opti.variable();    % Duration of control
ctrl.u  = opti.variable(1, N_ctrl); % Control values

params.m1 = 1;
params.m2 = 1;
params.k = 100;
params.d = 0.1;
params.g = 9.81;
params.xd = 0.5;

z0 = [0.5; 1; 0 ;0];

[tout,zout] = stance_simulation_casadi(z0,ctrl,params,N_ctrl);
opti.minimize(-zout(3,N_ctrl));


% Add lower and upper bounds
opti.subject_to(ctrl.tf >= 0.1);
opti.subject_to(ctrl.tf <= 0.6);
opti.subject_to(ctrl.u >= -2*ones(1, N_ctrl));
opti.subject_to(ctrl.u <=  2*ones(1, N_ctrl));

% Can't extend the spring too much
for i = 1:N_ctrl
    opti.subject_to(zout(1,i) - zout(2,i) <= 1);
    opti.subject_to(zout(1,i) - zout(2,i) >= -1);
end

p_opts = struct('expand',false);
opti.solver('ipopt',p_opts);

% Initial guess
opti.set_initial(ctrl.tf,0.35);
u_initial_guess = ones(1,N_ctrl);
opti.set_initial(ctrl.u, u_initial_guess);

% Solve the Optimization
sol = opti.solve();



%% Parse solution
tf = sol.value(ctrl.tf)+0.5;          % simulation final time
optimal_ctrl.tf = sol.value(ctrl.tf); % control final time
optimal_ctrl.u  = sol.value(ctrl.u);  % control values


%% Step 5: Simulate and Visualize the Result (same as before mostly)
[t, q] = hybrid_simulation_sol(z0,optimal_ctrl,params,N_ctrl);

figure(1)
subplot(1,2,1)
plot(t,q(1,:))
hold on
plot(t,q(2,:))
xlabel('time (s)')
ylabel('CoM Height (m)')
title('i should really go to class')
hold off 

subplot(1,2,2)
plot(t, q(3,:))
hold on
plot(t, q(4,:))
xlabel('time (s)')
ylabel('Vertical Velocity (m/s)')
title('i should really go to sleep')
hold off 

figure(2)  % u trajectory
plot(t, optimal_ctrl.u);
xlabel('time (s)')
ylabel('torque (Nm)')
title('yurrrrrrr')
hold off

%%
% Run the animation
m = gen_movie(t, q);
movie(m)
