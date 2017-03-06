[t,z] = ode45( @dyn, 0:0.01:15, [2 3 pi/6] );
figure
plot( z(:,1), z(:,2) )
grid on
figure
plot( t, z )
grid on
