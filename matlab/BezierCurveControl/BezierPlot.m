function Bp = BezierPlot(Bezier)


n=Bezier.n;
div=Bezier.div;
h=Bezier.h;
hnew=Bezier.hnew;
pl1=Bezier.pl1;
pl2=Bezier.pl2;
for j = 1:n %plots the points
        
    Bp(j,:)= getPosition(h(j));
 
end

n1=n-1; 

for    i=0:1:n1
sigma(i+1)=factorial(n1)/(factorial(i)*factorial(n1-i));  % for calculating (x!/(y!(x-y)!)) values 
    end

l=[];
UB=[];
for u=0:1/div:1
for d=1:n
UB(d)=sigma(d)*((1-u)^(n-d))*(u^(d-1));
end
l=cat(1,l,UB);                                      %catenation 
end

P=l*Bp;

set(pl1,'Xdata',P(:,1),'Ydata',P(:,2));

set(pl2,'Xdata',Bp(:,1),'Ydata',Bp(:,2));

end 