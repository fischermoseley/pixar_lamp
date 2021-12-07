
% This is the file where you define Task space control gain,
% model parameters and time span. Enjoy!

clear; 
clc;
close all;
%Model parameters (two link robotic arm)

l = 1;
c1= .5;
c2= .5;
m1 = 2;
m2 = 2;

I1 = 2/3;
I2 = 2/3;


Fx=0;
Fy=0;
tau1=0;
tau2=0;
th10=0;
th20=0;
p   = [ l; m1; m2; I1; I2; th10; th20; tau1; tau2; Fx; Fy;]; % parameters
control.kp=800;
control.kd=20;
control.kgp= 4000;   %Virtual control stiffness in the Cartesian coordinate
control.kgd= 80;      %Virtual control damping in the Cartesian coordinate
control.Force= [0 0];

%Trajectory creation for Bezier curve

clc
w=input('Press 1 for entry through mouse or 2 for keyboard repectively-->');
n=input('Enter no. of points  ');

xmax = l*2;  %    input('Enter the x max value of workspace: ');
ymax = l*2;   %     input('Enter the y max value of workspace: '); 

n1=n-1;


div = input('Enter the number of segment of the curve: ');
tspan =[0 input('Enter the timespan (sec): ')];
fig1=figure(1);
    axis equal;
    axis([-xmax xmax -ymax ymax]);
    
% if you choose to input points in mouse click
 if w==1
hnew=[];

for j = 1:n %plots the points
        h(j)=impoint;
    Bp(j,:)= getPosition(h(j))
    %plot(p(j,1),p(j,2),'*');
    axis equal;
     axis([-xmax xmax -ymax ymax]);
    hold on;
end
end
    


% if you choose to input points in numbers
if w==2
[Bp]=input('Enter co-odinates of points within brackets ->[x1 y1;x2 y2;x3 y3;...;xn yn] ');

end



ll=[];
UB=[];
for u=0:1/div:1
for i=1:n
UB(i)=factorial(n-1)/(factorial(i-1)*factorial(n-i))*(u^(i-1))*((1-u)^(n-i)); % calculating bezeir curve trajectory
end
ll=cat(1,ll,UB);                                      %catenation 
end


P=ll*Bp;

pl1=plot(P(:,1),P(:,2),'k.','MarkerSize',6); hold on;  % Plot the 
pl2=plot(Bp(:,1),Bp(:,2),'r--');



if w==1
uistack(pl1,'bottom');  
uistack(pl2,'bottom');        

Bezier.n=n;
Bezier.div=div;
Bezier.h=h;
Bezier.hnew=hnew;
Bezier.pl1=pl1;
Bezier.pl2=pl2; 


 fDrawing=@(hnew) BezierPlot(Bezier);

for j = 1:n %plots the points
     h(j).addNewPositionCallback(fDrawing);% Everytime each control point(Bp) changes, fdrawing is called to update the curve
       
    end  

end


%You can choose to enter numbers of draw by your self




tfinal=tspan(end);


%button control handle is created to simulate the model
ui1=uicontrol(fig1,'style','pushbutton','String','simulate','FontSize',20,'BackgroundColor',[0.929412 0.229412 0.229412],'Position', [20 10 100 30],'Callback', 'simulateTwoLinkArmBezier(tspan,Bezier,p,control)');


