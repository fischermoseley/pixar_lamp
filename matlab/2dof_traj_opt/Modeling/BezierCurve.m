function u = BezierCurve(ctrl_pt, t)

n = length(ctrl_pt);
u = 0;
    for i = 1:n
        u = u + factorial(n-1)/(factorial(i-1) * factorial(n-i)) * t^(i-1)*(1-t)^(n-i)*ctrl_pt(i);
%         u = u + ctrl_pt(i) * nchoosek(n,i) * (t^i .* (1-t).^(n-i));?
    end
end