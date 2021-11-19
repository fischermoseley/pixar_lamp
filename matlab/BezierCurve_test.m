close all
clc
%% Bezier Curve Test

t = 0;
i = 1;

% Control Points (Please try many different ones)
% ctrl_pt = [0, 1.5, 0, 3, 2];
ctrl_pt = [0, 2, 2, 2, 0];

% The time locations of each control points
ctrl_pt_x = linspace(0,1, length(ctrl_pt));

for d = 0:0.01:1 
    u(i) = BezierCurve(ctrl_pt, d);
    t(i) = d;
    i = i+1;
end

figure
plot(t, u)
hold on
plot(ctrl_pt_x, ctrl_pt, 'o');
hold off


function u = BezierCurve(ctrl_pt, t)

n = length(ctrl_pt);
u = 0;
    for i = 1:n
        u = u + factorial(n-1)/(factorial(i-1) * factorial(n-i)) * t^(i-1)*(1-t)^(n-i)*ctrl_pt(i);
%         u = u + ctrl_pt(i) * nchoosek(n,i) * (t^i .* (1-t).^(n-i));?
    end
end