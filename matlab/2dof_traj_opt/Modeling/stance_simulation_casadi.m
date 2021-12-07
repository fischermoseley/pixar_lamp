function [tout,zout,uout] = stance_simulation_casadi(z0,ctrl,p,tf,N)
    % Declare casadi symbolic variables
    tout = casadi.MX(zeros(1,N.ctrl));
    zout = casadi.MX(zeros(6,N.ctrl));
    uout = casadi.MX(zeros(2,N.ctrl-1));

    % Time discretization
    tout(1) = 0;
    dt = ctrl.tf/(N.ctrl-1);
    for i = 2:N.ctrl
        tout(i) = tout(i-1) + dt;
    end
    
    % Initial state
    zout(:,1) = z0;

    % Simulate
    for i = 1:N.ctrl-1
        t = tout(i);
        % Dynamics timestep
        [dz, u] = dynamics_continuous(t,zout(:,i),ctrl,p);
        zout(:,i+1) = zout(:,i) + dz*dt;
        zout(4:6,i+1) = discrete_impact_contact(zout(:,i+1), p);
        zout(1:3,i+1) = zout(1:3,i) + zout(4:6, i+1)*dt;
        % Log control input
        uout(:,i) = u;
    end
    
end

%% Discrete Contact
function qdot = discrete_impact_contact(z,p)
    qdot = z(4:6);
    vE = z(4);

    J  = [1, 0, 0]; %changing from [1, 0] to [1,0,0], this is a SWAG
    M = A_jumping_leg(z,p);
    Ainv = inv(M);
    lambda_z = 1/(J * Ainv * J.');
    F_z = lambda_z*(0 - vE);
    qdot = qdot + Ainv*J.'*F_z;
end

%% Continuous dynamics
function [dz, u] = dynamics_continuous(t,z,ctrl,p)
    u = control_laws(t,ctrl);    % get controls at this instant

    A = A_jumping_leg(z,p);      % get full A matrix
    b = b_jumping_leg(z,u,0,p);  % get full b vector
   
    x = A\b;                     % solve system for accelerations (and possibly forces)

    dz = casadi.MX(zeros(6,1));
    dz(1:3,1) = z(4:6);          % assign velocities to time derivative of state vector
    dz(4:6,1) = x(1:3);          % assign accelerations to time derivative of state vector
end

%% Control
function u = control_laws(t, ctrl)
    u1 = BezierCurve(ctrl.T1, t/ctrl.tf);
    u2 = BezierCurve(ctrl.T2, t/ctrl.tf);
    u = [u1, u2];
end
