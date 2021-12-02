function derive_everything()
%% Define symbolics.
name = 'jumping_leg';
syms t y dy ddy th1 dth1 ddth1 th2 dth2 ddth2 tau1 tau2 Fy l1 l2 c1 c2 m1 m2 mh I1 I2 g real

q   = [y; th1; th2];        % generalized coordinates
dq  = [dy; dth1; dth2];     % first time derivatives
ddq = [ddy; ddth1; ddth2];  % second time derivatives
u   = [tau1, tau2];         % control forces and moments
Fc   = Fy;                  % constraint forces and moments
p   = [l1; l2; c1; c2; m1; m2; mh; I1; I2; g];  % parameters

%% Calculate kinematics.
ihat = [1; 0; 0];
jhat = [0; 1; 0];
khat = cross(ihat,jhat);
er1hat = cos(th1)*ihat + sin(th1) * jhat;
er2hat = cos(th1 + th2)*ihat + sin(th1 + th2) * jhat;

rf = y*jhat;
rcm1 = rf+c1*er1hat;
rk = rf+l1*er1hat;
rcm2 = rk + c2*er2hat;
rh = rk + l2*er2hat;
keypoints = [rh rk rf];

%% Compute energetics.
ddt = @(r) jacobian(r,[q;dq])*[dq;ddq];
drcm1 = ddt(rcm1);
drcm2 = ddt(rcm2);
drh   = ddt(rh);

% Kinetic energy
T1 = (1/2)*m1*dot(drcm1, drcm1) + (1/2)* I1 * dth1^2;
T2 = (1/2)*m2*dot(drcm2, drcm2) + (1/2)* I2 * dth2^2;
Th = (1/2)*mh*dot(drh, drh);
T = T1 + T2 + Th;

% Potential energy
V1 = m1*g*dot(rcm1, jhat);
V2 = m2*g*dot(rcm2, jhat);
Vh = mh*g*dot(rh, jhat);
V = V1 + V2 + Vh;

%% Sum generalized forces.

% F2Q calculates the contribution of a force to all generalized forces
% for forces, F is the force vector and r is the position vector of the
% point of force application
F2Q = @(F,r) simplify(jacobian(r,q)'*(F));

% M2Q calculates the contribution of a moment to all generalized forces
% M is the moment vector and w is the angular velocity vector of the
% body on which the moment acts
M2Q = @(M,w) simplify(jacobian(w,dq)'*(M));
Q_F = F2Q(Fy*jhat,rf);
Q_tau1 = M2Q(tau1*khat,dth1*khat);
Q_tau2 = M2Q(tau2*khat,dth2*khat); 
Q_tau2R= M2Q(-tau2*khat,dth1*khat);
Q = Q_F + Q_tau1 + Q_tau2 + Q_tau2R;

%% Other useful stuff.
% Calculate rcm, the location of the center of mass
rcm = (m1*rcm1 + m2*rcm2 + mh*rh)/(m1+m2+mh);

% Assemble C, the set of constraints
C = y;  % When y = 0, the constraint is satisfied because foot is on the ground
dC = ddt(C);

%% Compute Energy and Equations of Motion
E = T+V;
L = T-V;
eom = ddt(jacobian(L,dq)') - jacobian(L,q)' - Q;  % form the dynamics equations

size(eom)

% Rearrange Equations of Motion.
A = jacobian(eom,ddq);
b = A*ddq - eom;

z = sym(zeros(length([q;dq]),1)); % initialize the state vector
z(1:3,1) = q;
z(4:6,1) = dq;

%% Export to functions.
directory = '../AutoDerived/';
% Evaluate total energy given the current state and parameters
matlabFunction(E,'file',[directory 'energy_' name],'vars',{z p});
% Evaluate A matrix given the current state and parameters
matlabFunction(A,'file',[directory 'A_' name],'vars',{z p});
% Evaluate b vector given the current state, current control, and parameters
matlabFunction(b,'file',[directory 'b_' name],'vars',{z u Fc p});

matlabFunction(keypoints,'file',[directory 'keypoints_' name],'vars',{z p});

matlabFunction(C,'file',[directory 'C_' name],'vars',{z u p});
matlabFunction(dC,'file',[directory 'dC_' name],'vars',{z u p});

% Evaluate the X and Y coordinates and speeds of the center of mass given the current state and parameters
drcm = ddt(rcm);             % Calculate center of mass velocity vector
COM = [rcm(1:2); drcm(1:2)]; % Concatenate x and y coordinates and speeds of center of mass in array
matlabFunction(COM,'file',[directory 'COM_' name],'vars',{z p});
