function f=Draw_Singular_state(x,q,x_value,deg)
% This function draw the current state of the system.
% x is the tool position [x [m], y [m], phi [rad]].
% q is the joint position [teta1, teta2, d3]
% x_value is the x that we iterate over
% deg is the direction of the free motion


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
h1=plot([a1(1) R*cos(q(1))],[a1(2) R*sin(q(1))],'LineWidth',3);
h2=plot([a2(1) R*cos(q(2))],[a2(2) R*sin(q(2))],'LineWidth',3);
h3=plot([a3(1) 0],[a3(2) -R],'LineWidth',3);



% Calculate the middle point of the plate
mid_point = [mean([a1(1), a2(1), a3(1)]), mean([a1(2), a2(2), a3(2)])];

% Calculate the end point of the arrow
arrow_length = 1.4;
arrow_end_point = mid_point + arrow_length * [cosd(deg), sind(deg)];

% Draw the arrow
arrow=quiver(mid_point(1), mid_point(2), arrow_end_point(1) - mid_point(1), arrow_end_point(2) - mid_point(2), 'LineWidth', 3, 'MaxHeadSize', 1.5, 'Color', 'blue');
% legend(arrow, 'Arrow Label');

txt= strcat('Singularity for x= ',string(x_value),' ,at angle of: ',string(deg),'\circ');
title(txt)
q1=strcat('\theta_1=',num2str((rad2deg(q(1)))),'\circ');
q2=strcat('\theta_2=',num2str((rad2deg(q(2)))),'\circ');
d3=strcat('d3=',num2str(round(q(3),3)));
%     arror=strcat('sin_angle=',num2str((deg)),'\circ');
% legend(q1,q2,d3,'EdgeColor',[1 1 1 ])
legend([h1, h2, h3], q1,q2,d3);


axis equal
% hold off