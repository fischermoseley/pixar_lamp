clear all; close all; clc;
setpath
addpath(genpath('casadi'))

N_ctrl  = 100; % number of dynamics timesteps where ctrl is applied
params.m1 = 1;
params.m2 = 1;
params.k = 100;
params.d = 0.1;
params.g = 9.81;
params.xd = 0.5;

ctrl.tf = 1;
ctrl.u = zeros(1, N_ctrl);

z0 = [1.5; 1; 0 ;0];
%% Step 5: Simulate and Visualize the Result (same as before mostly)
[t, q] = hybrid_simulation_sol(z0,ctrl,params,N_ctrl);

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
plot(t, ctrl.u);
xlabel('time (s)')
ylabel('torque (Nm)')
title('yurrrrrrr')

% Run the animation
figure(3)
m = gen_movie(t, q);
movie(m, 1, 50);