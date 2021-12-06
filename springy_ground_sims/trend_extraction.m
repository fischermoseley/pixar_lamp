% Extract trends from trajectory optimization with springy contact with the
% ground
clear all 
clc
close all

for k = 1:10
    com(k) = slim_optimization(k, 0.1);
    disp(com(k))
end

figure()
plot(com)