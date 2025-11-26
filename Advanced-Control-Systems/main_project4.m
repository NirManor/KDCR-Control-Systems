clear;clc;close all

global l1 l2 l l4 T dt xA yA zA xB yB zB m1 m2 m3 M g tau elbows qd qd_dot qd_dot2 controller qd_t qd_dot_t qd_dot2_t Kp Kd Ki K Beta delta gama F P t_vec

% controller=PDINV,PDG,PID,MINMAX,AC ======================================
controller='PDINV';
% manual change above =====================================================

% M = 0.5,0 ===============================================================
M = 0.5;
% manual change above =====================================================

% Set parameter
l1 = 0.4; l2 = 0.15; l = 0.6; l4 = 0.1;

m1 = 7800*l1*pi*0.015^2/4;
m2 = 7800*l2*pi*0.015^2/4;
m3 = 7800*l*pi*0.015^2/4;

xA=0.2; yA=0; zA=0.5;
xB=-0.3; yB=-0.2; zB=0.7;

g=9.81;

% dt = 0.0001 as required in HW3 / elbow 1 -1 is used in PD================
T=2; dt=0.0001; t=0:dt:T;
elbows = [1 -1];
% manual change above =====================================================

tau=tau_plan('pol',t);

qd=q_plan('pol',t);
qd_dot=q_dot_plan('pol',t,'diff');
qd_dot2=q_dot2_plan('pol',t,'diff');
x=x_plan('pol',t)';

% For an error in the initial position of the tool of 1cm upwards
zA=zA+0.01;
q0=q_plan('pol',t);

% Perform simulation for a time of 3s,after 2𝑠 the tool stay at point B at rest
T=3;
t=0:dt:T;

xd=[x;ones(length(t)-length(x),3)*[x(end,1) 0 0;0 x(end,2) 0;0 0 x(end,3)]];
qd=[qd;ones(length(t)-length(qd),3)*[qd(end,1) 0 0;0 qd(end,2) 0;0 0 qd(end,3)]];
qd_dot=[qd_dot;zeros(length(t)-length(qd_dot),3)];
qd_dot2=[qd_dot2;zeros(length(t)-length(qd_dot2),3)];

% tspan=0:dt:T;
tspan=[0 T];

option=odeset('RelTol',1e-5,'AbsTol',1e-9,'MaxStep',1e-2);

% The value for Kp, Kd, Ki and other parameter can change maually below
% Note that the function "state_eq" will change according to ''controller''
% Note that functions "state_eq" & "tau_cont" is coupled!!!
switch controller
    case 'PDINV'
        y0=[q0(1,:) 0 0 0 0 0 0 0];
        % manual change here ==============================================
        Kp=[1 0 0;0 6 0;0 0 5];
        Kd=[1 0 0;0 4 0;0 0 3];

%         Kp=[1 0 0;0 5.9 0;0 0 4.7];
%         Kd=[1 0 0;0 3.8 0;0 0 3.8];
        % manual change above =============================================
        [t,y]=ode45(@state_eq,tspan,y0,option);
    case 'PDG'
        y0=[q0(1,:) 0 0 0 0 0 0 0];
        % manual change here ==============================================
        Kp=1.5*[825 0 0;0 1200 0;0 0 900];
        Kd=1.5*[750 0 0;0 750 0;0 0 450];
                
        % manual change above =============================================
        [t,y]=ode45(@state_eq,tspan,y0,option);
    case 'PID'
        y0=[0 0 0 q0(1,:) 0 0 0 0 0 0 0 0 0 0];
        % manual change here ==============================================


        Kp=[1500 0 0;0 4000 0;0 0 16000];
        Ki=[1000 0 0;0 2000 0;0 0 4000];
        Kd=[1750 0 0;0 420 0;0 0 700];


        % manual change above =============================================
        [t,y_tilde]=ode45(@state_eq,tspan,y0,option);      
        y=y_tilde(:,4:13);
    case 'MINMAX'
        y0=[q0(1,:) 0 0 0 0 0 0 0];
        % manual change here ==============================================
        K=10*[1 0 0;0 1 0;0 0 1];
        Beta=1.05;
        delta=0.001;
        P=1*[1 0 0;0 1 0;0 0 1];
        % manual change above =============================================
        [t,y]=ode45(@state_eq,tspan,y0);
    case 'AC'
        y0=[q0(1,:) 0 0 0 0 0 0 0];
        % manual change here ==============================================
        Kp=4500*[3 0 0;0 3 0;0 0 1];
        Kd=1500*[1 0 0;0 3 0;0 0 1];
        Q=20*eye(6);
        gama=0.02;
        A=[zeros(3) eye(3);-Kp -Kd];
        F=sylvester(A',A,-Q);
        % manual change above =============================================
        [t,y]=ode45(@state_eq,tspan,y0,option);
end

% Calcalate the control torques/forces
t_vec=0:dt:((length(qd)-1)*dt);

qd_t=zeros(length(t),3);
qd_dot_t=zeros(length(t),3);
qd_dot2_t=zeros(length(t),3);

qd_t(:,1)=(interp1(t_vec,qd(:,1)',t))';
qd_t(:,2)=(interp1(t_vec,qd(:,2)',t))';
qd_t(:,3)=(interp1(t_vec,qd(:,3)',t))';

qd_dot_t(:,1)=(interp1(t_vec,qd_dot(:,1)',t))';
qd_dot_t(:,2)=(interp1(t_vec,qd_dot(:,2)',t))';
qd_dot_t(:,3)=(interp1(t_vec,qd_dot(:,3)',t))';

qd_dot2_t(:,1)=(interp1(t_vec,qd_dot2(:,1)',t))';
qd_dot2_t(:,2)=(interp1(t_vec,qd_dot2(:,2)',t))';
qd_dot2_t(:,3)=(interp1(t_vec,qd_dot2(:,3)',t))';

% tau_control=tau_cont(y(:,1:3)',y(:,4:6)',t,controller)';

%%
% Plot error in joint values
figure("Name",'Error in joint values w.r.t the planned motion')
subplot(311)
plot(t,(y(:,1)-qd_t(:,1)).*180/pi,'LineWidth',4);
title('Error in joint values w.r.t the planned motion')
xlabel('time [s]')
ylabel('\theta_1 [deg]')
set(gca,'FontSize',20)

subplot(312)
plot(t,(y(:,2)-qd_t(:,2)).*180/pi,'LineWidth',4);
xlabel('time [s]')
ylabel('\theta_2 [deg]')
set(gca,'FontSize',20)

subplot(313)
plot(t,(y(:,3)-qd_t(:,3)),'LineWidth',4);
xlabel('time [s]')
ylabel('d_3 [m]')
set(gca,'FontSize',20)

% Calculate tracking error
xd_t(:,1)=(interp1(t_vec',xd(:,1),t))';
xd_t(:,2)=(interp1(t_vec',xd(:,2),t))';
xd_t(:,3)=(interp1(t_vec',xd(:,3),t))';
x=forward_kin(y(:,1:3))';

PathLength=((xA-xB)^2+(yA-yB)^2+(zA-zB)^2)^0.5;

TrackingError=100*(((x(:,1)-xd_t(:,1)).^2+(x(:,2)-xd_t(:,2)).^2+(x(:,3)-xd_t(:,3)).^2).^0.5)/PathLength;

% Plot norm of the error in tool position
figure("Name",'Norm of error in tool position as % of total path length')
plot(t,TrackingError,'LineWidth',4)
hold on
plot([0 3],[0.15 0.15],'LineWidth',4)
legend('Magnitude of Tracking Error','0.15% Error requirement')
title('Norm of error in tool position as % of total path length')
xlabel('time [s]')
ylabel('Norm of the error [%]')
set(gca,'FontSize',20)

% Report of the results
fprintf("\nEnd of TrackingError :\n\n   %f",TrackingError(end));
[~,Index1p5]=min(abs(t-1.5));
fprintf("\n\nMin in 1.5 sec :\n\n   %f\n\n",TrackingError(Index1p5));

tau_control=zeros(length(t)-1,3);
tau_control(:,1)=diff(y(:,8))./diff(t);
tau_control(:,2)=diff(y(:,9))./diff(t);
tau_control(:,3)=diff(y(:,10))./diff(t);
t_graph=t(1:end-1);


% Plot control torques and forces 
figure("Name",'Control torques/forces compared to the planned torques/forces')
tau_plan=[tau';ones(T/dt-2/dt,1)*tau(:,end)'];
subplot(311)
plot(t_graph,tau_control(:,1),'LineWidth',4)
hold on
plot(t_vec,tau_plan(:,1),'r:','LineWidth',6)
hold off
title('Control torques/forces compared to the planned torques/forces')
xlabel('time [s]')
ylabel({'Torque on';' 1st joint [Nm]'})
legend('Control','Planned')
set(gca,'FontSize',18)

subplot(312)
plot(t_graph,tau_control(:,2),'LineWidth',4)
hold on
plot(t_vec,tau_plan(:,2),'r:','LineWidth',6)
hold off
xlabel('time [s]')
ylabel({'Torque on';'2nd joint [Nm]'})
legend('Control','Planned')
set(gca,'FontSize',18)

subplot(313)
plot(t_graph,tau_control(:,3),'LineWidth',4)
hold on
plot(t_vec,tau_plan(:,3),'r:','LineWidth',6)
hold off
xlabel('time [s]')
ylabel({'Force on'; '3rd joint [N]'})
legend('Control','Planned')
set(gca,'FontSize',18)

