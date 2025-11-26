%% Q1 - Inverse kinematics

close all;clear all;clc

syms x_a1 x_a2 x_a3 y_a1 y_a2 y_a3


global r L R
r=2; L=3.5; R=4;

% x_a= [-3,-2,45]
% x_b=[-2,0,0]
x=[-3,-1,45];
Draw_inverse_kin(x)


%% Q2 - Forward kinematics
close all;clear all;clc

% Initialize error matrix

% Q = [-2.4530 -1.8237 4.6904 ; 1.9351 -1.8237 4.6904 ; -2.4530 2.4672 4.6904 ;1.9351 2.4672 4.6904];
elbows=[1,1,1; -1,1,1; 1,-1,1 ;-1,-1,1];
global r L R 
r=2; L=3.5; R=4;   
Pos=[-3,-1,pi/4];
elbow = elbows(1,:);
q=inverse_kin(Pos, elbow)
q_vla=[rad2deg(q(1)),rad2deg(q(2)),q(3)]
Draw_state(Pos,q);
str="reference: "+strcat("X=",string(Pos(1)),", Y=",string(Pos(2)),", \phi=",string(Pos(3)),'\circ');
title(str);
figure
    
X = forward_kin(q);
X = double(X)

q1 = inverse_kin(X(1,:), elbow);
q2 = inverse_kin(X(2,:), elbow);
q3 = inverse_kin(X(3,:), elbow);
q4 = inverse_kin(X(4,:), elbow);

Draw_state(X(1,:),q1);
str="Solution 1: "+strcat("X=",string(X(1,1)),", Y=",string(X(1,2)),", \phi=",string(X(1,3)),'\circ');
title(str);
figure
Draw_state(X(2,:),q2);
str="Solution 2: "+strcat("X=",string(X(2,1)),", Y=",string(X(2,2)),", \phi=",string(X(2,3)),'\circ');
title(str)
figure
Draw_state(X(3,:),q3);
str="Solution 3: "+strcat("X=",string(X(3,1)),", Y=",string(X(3,2)),", \phi=",string(X(3,3)),'\circ');
title(str)
figure
Draw_state(X(4,:),q4);
str="Solution 4: "+strcat("X=",string(X(4,1)),", Y=",string(X(4,2)),", \phi=",string(X(4,3)),'\circ');
title(str)
fprintf('solution %i:\n', i)

ANS = q;
Plate_pose=[-3,-1,pi/4]
e1 = ANS - q1
e2 = ANS - q2
e3 = ANS - q3
e4 = ANS - q4

e1_plate = X(1,:)-Plate_pose
e2_plate = X(2,:)-Plate_pose
e3_plate = X(3,:)-Plate_pose
e4_plate = X(4,:)-Plate_pose
    
%% Q3 - Path planing

close all;clear all;clc

% Constants
global T dt xA yA phiA xB yB phiB r L R
% Initial and final positions (in meters): 2023
xA = -3; yA=-2; phiA=pi/4; 
xB=-2; yB=0; phiB=0;

% The parameters of the simulation (in sec.)
T=2; dt=0.005;

% The time vector:
t=0:dt:T;
r=2; L=3.5; R=4;
% r=1.5; L=3; R=3.5;
x = pos_plan(t);
q=joint_plan(t);


figure

plot(t,rad2deg(q(1,:)),'LineWidth',3)
grid on
hold on
plot(t,rad2deg(q(2,:)),'LineWidth',3)
% plot(t,(q(3,:)),'LineWidth',3)
legend('\theta_1','\theta_2')
xlabel('Time [sec]')
ylabel('Joint Positions [deg]')

figure
plot(t,(q(3,:)),'LineWidth',3)
xlabel('Time [sec]')
ylabel('d3 Position')

figure
Draw_state(x(:,1),[q(1,1) q(2,1) q(3,1)]);
Draw_state(x(:,101),[q(1,101) q(2,101) q(3,101)]);
Draw_state(x(:,201),[q(1,201) q(2,201) q(3,201)]);
Draw_state(x(:,301),[q(1,301) q(2,301) q(3,301)]);
Draw_state(x(:,401),[q(1,401) q(2,401) q(3,401)]);


% Plot trajectories  animation

% figure
% % Loop through all the instances of the path and update the plot
% for i = 2:length(t)
%     clf;
%     % Update the plot with the new state
%     Draw_state(x(:,i),[q(1,i) q(2,i) q(3,i)]);
%     % Pause for a short time to create animation effect
%     pause(0.001)
% end


%% Q4 - Calculate the Jacobian

close all;clear all;clc
syms x y r phi L R theta1 theta2 d3

[J_x, J_q] = Jacobian_calculation(x, y, r, phi, L, R, theta1, theta2, d3)

det_J_x=simplify(det(J_x))
det_J_q=simplify(det(J_q))

%% Q5 - Finding singularities points

close all;clear all;clc
% This code find the singularity of Jx for a given elbow status

global R r L

% The robot's dimensions
R=4; r=2; L=3.5; 

% Vector of the position in x dir.
x=0.5:1e-4:2;
% Unit convert from deg to rad
phi=10*pi/180;
y=0;
% The elbow status:
elbow=[-1 -1 1];

for i=1:length(x)
    % The tool position
    
    X=[x(i) y phi];
    % Calculate q vector from inverse kinematics  1.2446
    Q=inverse_kin(X,elbow);
    % Calculate the jacobian Jx
    Jx=Jacobian_x(X,Q);
    % Calculate the Jacobian determinant
    Jx_det(i)=det(Jx);
end
% Finding the singularity point
[value,index]=min(abs(Jx_det));
x(index)
figure
plot(x,Jx_det)
hold on
plot(x(index),Jx_det(index),'o')
xlabel('X') 
ylabel('Det (J_x)') 
% hold off
% The tool position at the singularity point
X=[x(index) y phi];
% Calculate q vector from inverse kinematics at the singularity point
Q=inverse_kin(X,elbow)
% Calculate the jacobian Jx at the singularity point
Jx=Jacobian_x(X,Q)
% Calculate the Jacobian determinant at the singularity point
Jx_det=det(Jx)
% Calculate eigenvalues and eigenvectors
[A,B]=eig(Jx)
% Calculate the movement angle [deg]
Sin_deg=atan2(A(2,:),A(1,:))*180/pi

figure
Draw_Singular_state(X,Q,x(index),Sin_deg(2));
% Calculate the Jacobian rank
rank(Jx)