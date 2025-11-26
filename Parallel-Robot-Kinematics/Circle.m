function f=Circle(R,x0,y0,res)
% This function draw circle with radius R with center at x0,y0. res define
% the resulotion of the draw
Angle=0:2*pi/res:2*pi;
x=R*cos(Angle);
y=R*sin(Angle);
plot(x+x0,y+y0,'LineWidth',4);
f=1;