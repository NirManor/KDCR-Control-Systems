function Jx=Jacobian_x(X,Q)
% This function calculate the Jacobian Jx according the tool position X and
% the actuators position Q.
% X is vector with the following format: [x, y, phi]
% Q is vector with the following format: [teta1, teta2, teta3]
global R r
x=X(1);
y=X(2);
phi=X(3);
teta1=Q(1);
teta2=Q(2);
d3=Q(3);
s1=sin(teta1);
c1=cos(teta1);
s2=sin(teta2);
c2=cos(teta2);

Jx=zeros(3);
Jx(1,1)=2*x+2*r*cos(phi+pi/3)-2*R*c1;
Jx(1,2)=2*y+2*r*sin(phi+pi/3)-2*R*s1;
Jx(1,3)=2*r*((sin(phi+pi/3))*(R*c1-x)+(cos(phi+pi/3))*(-R*s1+y));
Jx(2,1)=2*x-2*R*c2;
Jx(2,2)=2*y-2*R*s2;
Jx(3,1)=2*x+2*r*cos(phi);
Jx(3,2)=2*y-2*R+2*r*sin(phi);
Jx(3,3)=-2*r*(R*cos(phi) - y*cos(phi) + x*sin(phi));

