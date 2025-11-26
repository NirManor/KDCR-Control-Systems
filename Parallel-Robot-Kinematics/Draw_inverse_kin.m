function f=Draw_inverse_kin(x)
% This function draw the 8 solution of the inverse kinematics.
% x is the tool position [x [m], y [m], phi [deg]]

% The robot's parameters
global r L R
r=2; L=3.5; R=4;

close all;
% Drawing resulation
res=1000;

% Convert the phi units from deg to rad
x(3)=x(3)*pi/180;

% The 8 configurations are:
elbows=[1,1,1; -1,1,1; 1,-1,1 ;-1,-1,1];
for i=1:4
    elbow=elbows(i,:);
    % The inverse kinematics to solve the actuators' position
    q=inverse_kin(x',elbow);
    fprintf('option %i:\n', i)
    p_value=(q)
    fprintf('q: %i\n', p_value)
    figure
    % Draw the circular track of radius R
    Circle(R,0,0,res);
    hold on
    % Draw the three revolute joints on the circular track
    Circle(0.1,R*cos(q(1)),R*sin(q(1)),100);
    Circle(0.1,R*cos(q(2)),R*sin(q(2)),100);
    Circle(0.1,0,-R,100);
    % Calculate the triangle plate edges
    a2=[x(1),x(2)];
    a3=[x(1)+r*cos(x(3)),x(2)+r*sin(x(3))];
    a1=[x(1)+r*cos(x(3)+pi/3),x(2)+r*sin(x(3)+pi/3)];
    % Draw the plate and the links
    patch([a1(1) a2(1) a3(1) a1(1)],[a1(2) a2(2) a3(2) a1(2)],'green','edgecolor','black')
    h1=plot([a1(1) R*cos(q(1))],[a1(2) R*sin(q(1))],'LineWidth',3);
    h2=plot([a2(1) R*cos(q(2))],[a2(2) R*sin(q(2))],'LineWidth',3);
    h3=plot([a3(1) 0],[a3(2) -R],'LineWidth',3);
    
    txt= strcat('Solution',num2str(i),' :');
    title(txt)
    q1=strcat('\theta_1=',num2str(round(q(1),3)),'\circ');
    q2=strcat('\theta_2=',num2str(round(q(2),3)),'\circ');
    d3=strcat('d3=',num2str(round(q(3),3)));
    legend([h1, h2, h3], q1,q2,d3);

    axis equal
    hold off
end