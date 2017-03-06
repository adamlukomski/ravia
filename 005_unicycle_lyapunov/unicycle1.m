function unicycle1

figure(100)
axis
hold on
for x = -5:5
    if x==5 | x==-5
        for y=-5:5
            [t,q] = ode45( @robo, 0:0.05:15, [x y 0] );
	    plot( q(:,1), q(:,2) )
	    pause(0.01)
        end
    else
        for y=-5:10:5
            [t,q] = ode45( @robo, 0:0.05:15, [x y 0] );
            plot( q(:,1), q(:,2) )
            pause(0.01)
        end
    end
end

function dq = robo( t,q )
t
    x = q(1);
    y = q(2);
theta = q(3);

    e = sqrt(x^2+y^2);
alpha = atan2(y,x)-theta;
 beta = alpha+theta;

v = -e*cos(alpha);
w = alpha + (alpha+beta)*sin(alpha)*cos(alpha)/alpha;

dq = [ v*cos(theta) ; ...
       v*sin(theta) ; ...
       w ];
