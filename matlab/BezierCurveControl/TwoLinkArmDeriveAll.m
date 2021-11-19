clear all
close all

name = 'TwoLinkArm';

% Define variables for time, generalized coordinates + derivatives, controls, and parameters 
syms t th1 dth1 ddth1 th2 dth2 ddth2 I1 I2 l m1 m2 th10 th20 tau1 tau2 Fx Fy u d real; 

% Group them
q   = [th1 th2]';      % generalized coordinates
dq  = [dth1 dth2]';    % first time derivatives
ddq = [ddth1 ddth2]';  % second time derivatives
u   = [tau1 tau2 Fx Fy]';     % controls
p   = [ l; m1; m2; I1; I2; th10; th20; tau1; tau2; Fx; Fy;];        % parameters 

% Generate Position/velocity Vectors 
ihat = [1; 0; 0];
jhat = [0; 1; 0];
khat = cross(ihat,jhat);
er1hat =  cos(th1)*ihat + sin(th1)*jhat;
er2hat =  cos(th2)*ihat + sin(th2)*jhat;

rG= [0;0];
rA = l*er1hat;
rB = rA+l*er2hat;
rc1= 0.5*1*er1hat;
rc2 = rA+0.5*1*er2hat;

ddt = @(r) jacobian(r,[q;dq])*[dq;ddq]; % a handy anonymous function for taking time derivatives


drA = ddt(rA);
drB = ddt(rB);
drc1= ddt(rc1);
drc2 = ddt(rc2);

% Calculate Kinetic Energy, Potential Energy, and Generalized Forces
F2Q = @(F,r) simplify(jacobian(r,q)'*(F));    % force contributions to generalized forces
M2Q = @(M,w) simplify(jacobian(w,dq)'*(M));   % moment contributions to generalized forces

T = (1/2)*m1*dot(drc1, drc1)+(1/2)*m2*dot(drc2, drc2)+ 0.5*I1*dot(dth1*khat,dth1*khat)+0.5*I2*dot(dth2*khat,dth2*khat);
V = 0;
Q_tau1 = M2Q(tau1*khat,dth1*khat);
Q_tau2= M2Q(tau2*khat,dth2*khat);
Q_F = F2Q(Fx*ihat+Fy*jhat,rB);
Q=Q_tau1+Q_tau2 + Q_F;

% Calculate rcm, the location of the center of mass
rcm = (m1*rc1 + m2*rc2)/(m1+m2);

% Assemble R, the array of cartesian coordinates of the key points
R = [rG;rA(1:2); rB(1:2)];

%% All the work is done!  Just turn the crank...
% Derive Energy Function and Equations of Motion
E = T+V;
L = T-V;
g = ddt(jacobian(L,dq)') - jacobian(L,q)' - Q;
J= jacobian(rB,q);
dJ=ddt(J);
% Rearrange Equations of Motion
A = jacobian(g,ddq);
b = A*ddq - g;
ddq2 = A\b;

V=g+Q-A*ddq
% Calculate COM derivative
drcm = ddt(rcm);
COM = [rcm(1:2); drcm(1:2)];



% Write Energy Function and Equations of Motion
Nz = length(dq)*2;
z(1:2:Nz,1) = q;
z(2:2:Nz,1) = dq;
dz(1:2:Nz,1) = dq;
dz(2:2:Nz,1) = ddq2;



%UB(d)=sigma(d)*((1-u)^(n-d))*(u^(d-1));

matlabFunction(dz,'file',['f_' name],'vars',{t z u p});
matlabFunction(E,'file',['energy_' name],'vars',{z p});
matlabFunction(COM,'file',['COM_' name],'vars',{z p});
matlabFunction(R,'file',['z2R_' name],'vars',{z p});
matlabFunction(A,'file',['A_' name],'vars',{z p});
matlabFunction(V,'file',['V'],'vars',{z p});

