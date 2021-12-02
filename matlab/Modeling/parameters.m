function p = parameters() 
 l1 = 0.1778; % 7 inches
 l2 = 0.2794; % 11 inches - this naming convention is strange, but it should be backwards. look at derive_everything.m to see what i mean
 m1  = .5;
 m2 = .5;
 mh = 5; % 11 pounds 
 I1 = m1*l1^2/12;
 I2 = m2*l2^2/12;
 c1 = l1/2;
 c2 = l2/2;
 g = 9.81;
 p = [l1; l2; c1; c2; m1; m2; mh; I1; I2; g];
end