function test_formation1()

[t,q] = ode45( @dyn, 0:0.01:5, [ 3 5 pi/3 1 , 4 6 pi/3 0.1 , 7 8 pi/3 0.2 ] );
plot( q(:,[1 1+4 1+8]), q(:,[2 2+4 2+8]) );


function dq = dyn(t,q)

% q - x1 y1 theta1 v1     x2 y2 theta2 v2 ...
q1 = q(1:4);
q2 = q(5:8);
q3 = q(9:12);

u = zeros(6,1);

% u = [ a1 w1  a2 w2  a3 w3 ]

uni1 = [ q1(4)*cos(q1(3)) ; q1(4)*sin(q1(3)) ; u(2) ; u(1) ];
uni2 = [ q2(4)*cos(q2(3)) ; q2(4)*sin(q2(3)) ; u(4) ; u(3) ];
uni3 = [ q3(4)*cos(q3(3)) ; q3(4)*sin(q3(3)) ; u(6) ; u(5) ];

dq = [ uni1 ; uni2 ; uni3 ];