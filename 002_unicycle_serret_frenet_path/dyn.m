function dz = dyn(t,z)

x = z(1);
y = z(2);
theta = z(3);

% polar coords

p = sqrt( x^2 + y^2 );
gamma = atan2(y,x) - theta + pi;
delta = gamma + theta;

k1 = 1;
k2 = 10;
k3 = 10;

v = k1 * p * cos(gamma);
w = k2*gamma + k1*(sin(gamma)*cos(gamma)/gamma)*(gamma+k3*delta);

dz = [ v * cos(theta); v * sin(theta) ; w ];
