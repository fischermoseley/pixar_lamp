function b = b_jumping_leg(in1,in2,Fy,in4)
%B_JUMPING_LEG
%    B = B_JUMPING_LEG(IN1,IN2,FY,IN4)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    02-Dec-2021 14:19:37

c1 = in4(3,:);
c2 = in4(4,:);
dth1 = in1(5,:);
dth2 = in1(6,:);
dy = in1(4,:);
g = in4(10,:);
l1 = in4(1,:);
l2 = in4(2,:);
m1 = in4(5,:);
m2 = in4(6,:);
mh = in4(7,:);
tau1 = in2(:,1);
tau2 = in2(:,2);
th1 = in1(2,:);
th2 = in1(3,:);
t2 = cos(th1);
t3 = sin(th1);
t4 = th1+th2;
t5 = c1.^2;
t6 = l1.*t2;
t7 = cos(t4);
t8 = l1.*t3;
t9 = sin(t4);
t10 = c1.*dth1.*t2;
t11 = c2.*t7;
t12 = l2.*t7;
t13 = c2.*t9;
t14 = l2.*t9;
t15 = dy+t10;
t16 = dth1.*t11;
t17 = dth2.*t11;
t18 = dth1.*t12;
t19 = dth2.*t12;
t20 = dth1.*t13;
t21 = dth2.*t13;
t22 = dth1.*t14;
t23 = dth2.*t14;
t26 = t6+t11;
t27 = t6+t12;
t28 = t8+t13;
t29 = t8+t14;
t24 = t21.*2.0;
t25 = t23.*2.0;
t30 = dth1.*t26;
t31 = dth1.*t27;
t32 = dth1.*t28;
t33 = dth1.*t29;
t34 = t16+t17;
t35 = t18+t19;
t36 = t20+t21;
t37 = t22+t23;
t38 = t17+t30;
t39 = t19+t31;
t40 = t21+t32;
t41 = t23+t33;
t42 = dy+t38;
t43 = dy+t39;
t44 = t11.*t40.*2.0;
t45 = t12.*t41.*2.0;
t46 = t13.*t42.*2.0;
t47 = t14.*t43.*2.0;
t48 = -t46;
t49 = -t47;
mt1 = [Fy+dth2.*((m2.*(t20.*2.0+t24))./2.0+(mh.*(t22.*2.0+t25))./2.0)+dth1.*((m2.*(t24+t32.*2.0))./2.0+(mh.*(t25+t33.*2.0))./2.0+c1.*dth1.*m1.*t3)-g.*m1-g.*m2-g.*mh];
mt2 = [tau1-tau2-dth2.*((m2.*(t44+t48-t26.*t36.*2.0+t28.*t34.*2.0))./2.0+(mh.*(t45+t49-t27.*t37.*2.0+t29.*t35.*2.0))./2.0)-dth1.*((m2.*(t28.*t38.*2.0-t28.*t42.*2.0))./2.0+(mh.*(t29.*t39.*2.0-t29.*t43.*2.0))./2.0-(m1.*(c1.*t3.*t15.*2.0-dth1.*t2.*t3.*t5.*2.0))./2.0)-(m1.*(c1.*dth1.*t3.*t15.*2.0-dth1.^2.*t2.*t3.*t5.*2.0))./2.0+(m2.*(t38.*t40.*2.0-t40.*t42.*2.0))./2.0+(mh.*(t39.*t41.*2.0-t41.*t43.*2.0))./2.0-g.*m2.*t26-g.*mh.*t27-c1.*g.*m1.*t2];
mt3 = [tau2-dth2.*((m2.*(t44+t48-t11.*t36.*2.0+t13.*t34.*2.0))./2.0+(mh.*(t45+t49-t12.*t37.*2.0+t14.*t35.*2.0))./2.0)+(m2.*(t34.*t40.*2.0-t36.*t42.*2.0))./2.0+(mh.*(t35.*t41.*2.0-t37.*t43.*2.0))./2.0+dth1.*((m2.*(t46-t13.*t38.*2.0))./2.0+(mh.*(t47-t14.*t39.*2.0))./2.0)-g.*m2.*t11-g.*mh.*t12];
b = reshape([mt1,mt2,mt3],3,1);
