function simulateTwoLinkArmBezier(tspan,Bezier,p,control) 
%p   = [ l; m1; m2; I1; I2; th10; th20; tau1; tau2; Fx; Fy;];

n=Bezier.n;
div=Bezier.div;
h=Bezier.h;
hnew=Bezier.hnew;
pl1=Bezier.pl1;
pl2=Bezier.pl2; 

Bp=BezierPlot(Bezier);
xmax = p(1)*2;  %    input('Enter the x max value of workspace: ');
ymax = p(1)*2; 
tfinal=tspan(end);
%n=max(length(Bp));

%initial conditions
%solving inverse kinematics to find the initial position in the generalized coordinates. If it failed to
%find, adjust the initial guess. Currently  [1,-1]. 


th0=fsolve(@(th) [p(1)*cos(th(1)) + p(1)*cos(th(2)), p(1)*sin(th(1)) + p(1)*sin(th(2))]-1.*Bp(1,:), [1,-1]); 
%calculate initial velocity using deBezier curve function
if isempty(th0)
str1 = {'The solver failed to find the initial position by inverse kinematics, try other initial points (default is [1,-1]) in simulateTwoLinkArmBezier.m'};

text(0,1,str1)
end

    
    
for    i=1:n-1
    u=0;
sigma2(i,:)=factorial(n-2)/(factorial(i-1)*factorial(n-i-1))*((1-u)^(n-i-1))*(u^(i-1)).*((n-1).*(Bp(i+1,:)-1.*Bp(i,:)));  % for calculating (x!/(y!(x-y)!)) values 
end

dx=sum(sigma2,1)./(tfinal);


dx=sum(sigma2,1)./(tfinal);

%Jacobian
J=[ -p(1)*sin(th0(1)) -p(1)*sin(th0(2));
     p(1)*cos(th0(1))  p(1)*cos(th0(2))];
dth0=J\dx';
 


  % parameters

z0 = [th0(1),dth0(1),th0(2),dth0(2)];
opts = odeset('AbsTol',1e-12,'RelTol',1e-6);

%calling control function
f = @(t,z) f_TwoLinkArm_control_Cartesian_Bezier(t,tfinal,z,p,control,Bp,n);

sol = (ode45(f,tspan,z0,opts));

% figure(2)
% subplot(3,1,1)
% plot(sol.x,sol.y(2,:));     %plot Center of Mass
% subplot(3,1,2)
% COM = COM_TwoLinkArm(sol.y,p);
% plot(sol.x,COM(2,:));
% 
% subplot(3,1,3)
% plot(sol.x,energy_TwoLinkArm(sol.y,p))     %plot energy

R= z2R_TwoLinkArm(sol.y,p);%Calculating the end effector position
t = sol.x;
C = [1 2; 2 3];



animate(t,R,C)

  
 rB=R(5:6,:);               
 
pl3=plot(rB(1,:),rB(2,:),'.','MarkerSize',2); 
uistack(pl3,'bottom'); 
 axis([-xmax xmax -ymax ymax]);
 
 