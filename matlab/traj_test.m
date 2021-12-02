close all;
clear all;
l1 = 3;
l2 = 4;

% phi1_start = ;
% phi2_start = ;
%y_start = l2*(sin(phi2_start)/sin(phi1_start));

% phi1_end = ;
% phi2_end = ;
%y_end = l2*(sin(phi2_end)/sin(phi1_end));

%y_traj = linspace(y_start, y_end, 100);
y_traj = linspace(5,2,100);
phi1_traj = zeros(100,1);
phi2_traj = zeros(100,1);

for i=1:100
phi1_traj(i) = acos((l2^2-y_traj(i)^2-l1^2)/(-2*y_traj(i)*l1));
phi2_traj(i) = acos((y_traj(i)^2-l2^2-l1^2)/(-2*l2*l1));
end

figure()
%plot(y_traj)
hold on 
plot(phi1_traj)
plot(phi2_traj)
plot(l2*sin(phi2_traj)./sin(phi1_traj))

% i'm too lazy to make this automatically go to a header file, so you'll
% just have to copy paste for now chief, i'm so sorry.

th1_traj = sprintf('%.3f, ' , phi1_traj);
th1_traj = th1_traj(1:end-2); % strip final comma
th1_traj = strcat("float th1_trajectory[] = {",th1_traj,"};");

th2_traj = sprintf('%.3f, ' , phi2_traj);
th2_traj = th2_traj(1:end-2); % strip final comma
th2_traj = strcat("float th2_trajectory[] = {",th2_traj,"};");