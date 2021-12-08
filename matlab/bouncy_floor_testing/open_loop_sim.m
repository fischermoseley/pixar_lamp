% Simulate system with a fixed set of inputs, and plot results.
% No optimization performed here.

% Housekeeping
clear all; close all; clc;
addpath([pwd '/casadi'])
addpath(genpath('casadi'))

% Parameters
N_ctrl  = 1000; % number of dynamics timesteps where ctrl is applied
params.m1 = 1;
params.m2 = 1;
params.kl = 200;
params.dl = 0.1;
params.g = 9.81;
params.xd = 0.5;
params.kg = 400;
params.dg = 10;

ctrl.tf = 10;
ctrl.u = zeros(1, N_ctrl);

% Simulate
q0 = [1.5; 1; 0 ;0];
[t, q] = hybrid_simulation_sol(q0,ctrl,params,N_ctrl);

% Display results
figure(1)
subplot(1,2,1)
plot(t,q(1,:))
hold on
plot(t,q(2,:))
xlabel('Time (s)')
ylabel('Height (m)')
title('Leg Trajectory')
legend('Hip','Foot')
hold off 

subplot(1,2,2)
plot(t, q(3,:))
hold on
plot(t, q(4,:))
xlabel('time (s)')
ylabel('Velocity (m/s)')
title('Leg Trajectory')
legend('Hip','Foot')
hold off 

figure(2)
subplot(1,2,1)
plot(t, ctrl.u);
xlabel('Time (s)')
ylabel('Actuator Force (N)')

subplot(1,2,2)
plot(t, q(1,:) - q(2,:));
xlabel('Time (s)')
ylabel('Leg Separation (m)')

% Run the animation
figure(3)
m = gen_movie(t, q);
movie(m, 1, 50);