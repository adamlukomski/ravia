%
% the equations for a simple pendulum:
%    m a = sum of all the forces
%    m a = gravity + friction + input
%
%    gravity  = - m g sin( theta )
%    friction = - f v
%    input = u  (torque)
%            1/r u  (force)
%
%    m a = - m g sin( theta ) - f v + 1/r u
% 
%    linear to angular:
% 
%    v = w r
%    a = e r
% 
%    m r e = - m g sin( theta ) - f r w + 1/r u
%    e = -g/r sin(theta) -f/m w + 1/(m*r*r) u
%
%    state:
%    x1 = theta
%    x2 = w
%
%    dx1 = w = x2
%    dx2 = e = -g/r sin( x(1) ) -f/m x(2) + 1/(m*r*r) u
%  
%    m=1, g=9.81, f=1, r=1
%
%
%   dx = [ x(2) ; -sin(x(1))-x(2)+u ]
%

% simulation without input
ode45( @(t,x)  [ x(2) ; -sin(x(1))-x(2) ] , 0:0.01:10, [pi/3 0] )

pause(2)

% simulation with input
u = 3;
ode45( @(t,x,u)  [ x(2) ; -sin(x(1))-x(2)+u ] , 0:0.01:10, [pi/3 0] )
