function [t,q] = hybrid_simulation_sol(q0,ctrl,params,timesteps)
    % hello my fellow gamers it's time to riot
    t = zeros(1,timesteps);
    q = zeros(4,timesteps);
    u = ctrl.u;

    m1 = params.m1;
    m2 = params.m2;
    kl = params.kl;
    dl = params.dl;
    g  = params.g;
    xd = params.xd;
    kg = params.kg;
    dg = params.dg;
    
    % Discretize time
    dt = ctrl.tf/(timesteps-1);
    for i = 2:timesteps
        t(i) = t(i-1) + dt;
    end
    
    % Set initial state
    q(:,1) = q0;

    % Simulate
    for step = 2:timesteps
        x1 = q(1, step-1);
        x2 = q(2, step-1);
        dx1 = q(3, step-1);
        dx2 = q(4, step-1);
        
        %%% Compute velocities from accelerations
        % x2:
        %if (x2 < 0)
        %    q(4,step) = q(4, step-1) + ((-m2*g+kl*(x1-x2-xd) + dl*(dx1-dx2) - u(step-1) - kg*x2 - dg*dx2)/m2) * dt; 
        %else
        %    q(4,step) = q(4, step-1) + ((-m2*g+kl*(x1-x2-xd) + dl*(dx1-dx2) - u(step-1))/m2) * dt; 
        %end
        %
        % hi kids it's me pull up a chair by the fire and let papa read you
        % a story <3
        %
        % casadi uses symbolics to do optimization, just like pydrake.
        % this is probably to make autodiff and explicit gradient
        % calculations possible 
        %
        % we want to impose the springiness of the ground once x2 < 0.
        % however, this is bad for calculating gradients because it's
        % discontinuous, and it's worse for casadi because it literally
        % refuses to cast to a boolean
        %
        % as a result we use a logistical curve to smoothly add the
        % spring+damper force from the ground, which is the fg term. the
        % logistical curve is parameterized by epsilon here.
        
        epsilon = 40;
        fg = (-kg*x2 - dg*dx2) * exp(-epsilon*x2)/(exp(-epsilon*x2) - 1);
        q(4,step) = q(4, step-1) + ((-m2*g+kl*(x1-x2-xd) + dl*(dx1-dx2) - u(step-1) + fg)/m2) * dt;
        
        % x1:
        q(3,step) = q(3, step-1) + ((-m1*g-kl*(x1-x2-xd) - dl*(dx1-dx2) + u(step-1))/m1) * dt;
        
        %%% Compute positions from velocities
        q(1,step) = q(1, step-1) + q(3, step) * dt;
        q(2,step) = q(2, step-1) + q(4, step) * dt;
    end 
end