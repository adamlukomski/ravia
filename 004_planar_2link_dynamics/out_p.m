function p = out_p(theta1,theta2)
%OUT_P
%    P = OUT_P(THETA1,THETA2)

%    This function was generated by the Symbolic Math Toolbox version 5.8.
%    18-Jan-2017 20:56:59

t2 = sin(theta1);
t3 = cos(theta2);
t4 = cos(theta1);
t5 = sin(theta2);
t6 = t3-1.0;
p = reshape([0.0,0.0,0.0,1.0,0.0,-t2,t4,1.0,0.0,t2.*t3.*-2.0+t2.*t6-t4.*t5,-t2.*t5+t3.*t4.*2.0-t4.*t6,1.0],[4, 3]);
