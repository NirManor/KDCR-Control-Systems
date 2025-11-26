function f=Draw_state(x,q)
% This function draw the current state of the system.
% x is the tool position [x [m], y [m], phi [rad]].
% q is the joint position [teta1, teta2, d3]


global r L R
r=2; L=3.5; R=4;

res=1000; 

Circle(R,0,0,res);
hold on
Circle(0.1,R*cos(q(1)),R*sin(q(1)),100);
Circle(0.1,R*cos(q(2)),R*sin(q(2)),100);
Circle(0.1,0,-R,100);
a2=[x(1),x(2)];
a3=[x(1)+r*cos(x(3)),x(2)+r*sin(x(3))];
a1=[x(1)+r*cos(x(3)+pi/3),x(2)+r*sin(x(3)+pi/3)];
patch([a1(1) a2(1) a3(1) a1(1)],[a1(2) a2(2) a3(2) a1(2)],'green','edgecolor','red');
plot([a1(1) R*cos(q(1))],[a1(2) R*sin(q(1))],'LineWidth',3);
plot([a2(1) R*cos(q(2))],[a2(2) R*sin(q(2))],'LineWidth',3);
plot([a3(1) 0],[a3(2) -R],'LineWidth',3);
axis equal
% hold off