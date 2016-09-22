%Figure Eight Torus
Parameters: c = 1, -pi <= u, v <= pi
Equations: x = (cos(u)*( c + sin(v)*cos(u)) - (sin(2*v)*sin(u)/2))
           y = (sin(u)*(c + sin(v)*cos(u)) - (sin(2*v)*sin(u)/2))
           z = s(in(u)*sin(v)) + (cos(u)*sin(2*v)/2)
plot(x,y,z)
