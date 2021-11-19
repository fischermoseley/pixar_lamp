function eom = f_TwoLinkArm_control_Catesian(t,tfinal,z,p,control,Bp,n);
 





for    i=1:1:n
sigma(i)=factorial(n-1)/(factorial(i-1)*factorial(n-i));  % for calculating (x!/(y!(x-y)!)) values 
    end

l=[];
UB=[];
 u=t/tfinal;
for d=1:n
UB(d)=sigma(d)*((1-u)^(n-d))*(u^(d-1));
end
rB_desired=(UB*Bp)';                                      %catenation 




J=[ -p(1)*sin(z(1)) -p(1)*sin(z(3));
     p(1)*cos(z(1))  p(1)*cos(z(3))];
 R=z2R_TwoLinkArm(z,p);
 rB=R(5:6);
 
 Tau = J'*(control.kgp*(rB_desired-rB)-control.kgd*(J*z(2:2:4)));

%if (t>3)
%    control.Force(2)=-400;


 
u=[Tau(1) Tau(2) control.Force(1) control.Force(2)]';

eom=f_TwoLinkArm(t,z,u,p);
end