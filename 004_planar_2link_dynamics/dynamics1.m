function dynamics1

syms theta1 theta2 dtheta1 dtheta2 real
dtheta = [ dtheta1 ; dtheta2 ];
theta = [ theta1 ; theta2 ];

n=length(theta);

u1 = [0;0;0];
p10 = [0;0;1;1];
w1 = [1;0;0];

u2 = [0;0;1];
p20 = [0;0;2;1];
w2 = [1;0;0];

s1 = [ w1 ; -cross(w1,u1) ];
s2 = [ w2 ; -cross(w2,u2) ];

A1 = simplify( expm( hat(s1) * theta1 ) );
A2 = simplify( expm( hat(s2) * theta2 ) );

p1 = A1 * p10;
p2 = A1 * A2 * p20;

p = [ [0;0;0;1] p1 p2];

matlabFunction( p, 'file', 'out_p', 'vars', theta );

m = 1;
J = [ 1/12 0 0 ; 0 1/12 0 ; 0 0 1/12 ];
M = [ J zeros(3) ; zeros(3) m*eye(3) ];

gc01 = [ eye(3) [0;0;0.5] ; 0 0 0 1 ];
gc02 = [ eye(3) [0;0;1.5] ; 0 0 0 1 ];

pc1 = A1 * gc01 * [0;0;0;1];
pc2 = A1 * A2 * gc02 * [0;0;0;1];

gc1 = A1 * gc01;
gc2 = A1 * A2 * gc02;

dpc1 = simplify( hat(s1) * dtheta1 * A1 * gc01 );
dpc2 = simplify( hat(s1) * dtheta1 * A1 * A2 * gc02 + A1 * hat(s2) * dtheta2 * A2 * gc02 );
dpc1 = simplify( dpc1 )
dpc2 = simplify( dpc2 )
dpc1 = simplify( dpc1*inv(gc1) ) 
dpc2 = simplify( dpc2*inv(gc2) ) 
%  dpc1 = simplify( inv(gc1)*dpc1 ) * gc01
%  dpc2 = simplify( inv(gc2)*dpc2 ) * gc02
%  pause


J1 = jacobian( unhat(dpc1), dtheta )
J2 = jacobian( unhat(dpc2), dtheta )

% D = J1'*inv(Ad(gc01)')*M*Ad(gc01)*J1 + J2'*inv(Ad(gc02)')*M*Ad(gc02)*J2;
D = J1'*M*J1 + J2'*M*J2;
C = sym(0);
for i=1:n
    for j=1:n
        C(i,j) = sym(0);
        for k=1:n
            C(i,j) = C(i,j) + 0.5*(diff(D(i,j),theta(k)) + diff(D(i,k),theta(j)) - diff(D(k,j),theta(i)))*dtheta(k);
        end
    end
end
C = simplify(C)
D = simplify(D)
pause


V = m*9.81*pc1(3)+m*9.81*pc2(3);
G = jacobian( V, theta )';

matlabFunction( D, 'file', 'out_D', 'vars', [theta ; dtheta] );
matlabFunction( C, 'file', 'out_C', 'vars', [theta ; dtheta] );
matlabFunction( G, 'file', 'out_G', 'vars', [theta ; dtheta] );


E = 0.5*dtheta'*D*dtheta+V;
matlabFunction( E, 'file', 'out_E', 'vars', [theta ; dtheta] );

opts = odeset;
opts.RelTol = 1e-12;
[t,x] = ode23t( @robot, 0:0.01:10, [pi/6 pi/6 0 0 ], opts )
for i=1:length(t)
    Energy(i) = out_E( x(i,1), x(i,2), x(i,3), x(i,4) )
    p = out_p( x(i,1), x(i,2) );
    plot( p(2,:), p(3,:) )
    axis( [-2 2 -2 2] )
    pause(0.01)
end
Energy

function dx = robot(t,x)
q=x(1:2);
dq=x(3:4);

D = out_D( x(1), x(2), x(3), x(4) );
C = out_C( x(1), x(2), x(3), x(4) );
G = out_G( x(1), x(2), x(3), x(4) );

E = out_E( x(1), x(2), x(3), x(4) )

ddq = inv(D)*(-C*dq-G);
dx = [dq;ddq];