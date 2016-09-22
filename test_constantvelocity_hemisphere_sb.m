M=1000;
g=9.8;
r=0.4;
R=0.8;
theta_i=asin(r/(r+R));
F=[1 0;2 0;3 0;4 0;5 0;6 0;7 0;8 0;9 0;10 0];
Zpos=[1 0;2 0;3 0;4 0;5 0;6 0;7 0;8 0;9 0;10 0]
v=5;
mu=0.4;
dt=0.01;
k=11;
for theta=theta_i:dt:pi-theta_i
    N=(M*v*v/R)-(M*g*sin(theta));k=k+1;
    %Fy=(N*sin(theta))-(M*g)+(M*v*v/R*sin(theta))-(mu*N*cos(theta));
    F=[F;k N];
    Zpos=[Zpos;k R*sin(theta)];
    
end
F=[F;k+1 0;k+2 0;k+3 0;k+4 0;k+5 0;k+6 0;k+7 0;k+8 0;k+9 0;k+10 0];
Zpos=[Zpos;k+1 0;k+2 0;k+3 0;k+4 0;k+5 0;k+6 0;k+7 0;k+8 0;k+9 0;k+10 0];

