function p = parameters() 
 l = 0.5;
 m1  = .4*l;
 m2 = .4*l;
 mh = .1;
 I1 = m1*l^2/12;
 I2 = m2*l^2/12;
 c1 = 0.25;
 c2 = 0.25;
 g = 9.81;
 p = [l; c1; c2; m1; m2; mh; I1; I2; g];
end