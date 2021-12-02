function keypoints = keypoints_jumping_leg(in1,in2)
%KEYPOINTS_JUMPING_LEG
%    KEYPOINTS = KEYPOINTS_JUMPING_LEG(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    02-Dec-2021 14:19:37

l1 = in2(1,:);
l2 = in2(2,:);
th1 = in1(2,:);
th2 = in1(3,:);
y = in1(1,:);
t2 = cos(th1);
t3 = sin(th1);
t4 = th1+th2;
t5 = l1.*t2;
t6 = l1.*t3;
keypoints = reshape([t5+l2.*cos(t4),t6+y+l2.*sin(t4),0.0,t5,t6+y,0.0,0.0,y,0.0],[3,3]);
