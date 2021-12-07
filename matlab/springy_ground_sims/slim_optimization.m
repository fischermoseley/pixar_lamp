function max_com_height = slim_optimization(k_ground, d_ground) 
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

tf = NaN;
[tout,zout,uout] = stance_simulation_casadi(z0,ctrl,p,tf,N,k_ground, d_ground);

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

[t z u indices] = hybrid_simulation_sol(z0,optimal_ctrl,p,[0 tf], k_ground, d_ground); % run simulation
COM = COM_jumping_leg(z,p);
max_com_height = COM(4, N.ctrl);