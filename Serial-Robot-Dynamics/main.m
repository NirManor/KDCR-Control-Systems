clear
clc

global l1 l2 l l4 T dt xA yA zA xB yB zB m1 m2 m3 M g tau elbows

%% 2 Inverse dynamics

l1 = 0.4; l2 = 0.15; l = 0.6; l4 = 0.1;

m1 = 7800*l1*pi*0.015^2/4;
m2 = 7800*l2*pi*0.015^2/4;
m3 = 7800*l*pi*0.015^2/4;
M = 0.5;

xA=0.2; yA=0; zA=0.5;
xB=-0.2; yB=-0.2; zB=0.7;

g=9.81;

T=2; dt=0.001; t=0:dt:T;

elbows = [1 1];

% plot tau
tau=tau_plan('pol',t);
figure(2)
subplot(311)
plot(t,tau(1,:),'LineWidth',2)
xlabel('time [s]')
ylabel('torque [Nm]')
grid
title('\fontsize{22} \theta_1')
set(gca,'FontSize',16)
subplot(312)
plot(t,tau(2,:),'LineWidth',2)
xlabel('time [s]')
ylabel('torque [Nm]')
grid
title('\fontsize{22} \theta_2')
set(gca,'FontSize',16)
subplot(313)
plot(t,tau(3,:),'LineWidth',2)
xlabel('time [s]')
ylabel('force [N]')
grid
title('\fontsize{22} d_3')
set(gca,'FontSize',16)

% plot f2
f2y = f2_plan('pol',t);
figure(3)
plot(t,f2y,'LineWidth',4)
title('force on joint 2 in the direction of the joint axis')
xlabel('time [s]')
ylabel('force [N]')
grid
set(gca,'FontSize',16)

%% 4 Forward dynamics

q=q_plan('pol',t);
q_dot=q_dot_plan('pol',t,'diff');
tspan=0:0.0001:2;
y0=[q(1,:) 0 0 0];
option=odeset('RelTol',1e-8,'AbsTol',1e-8);
[t,y]=ode45(@state_eq,tspan,y0,option);

figure(4)

yyaxis left
h1 = plot(t,y(:,1).*180/pi, 'r-','LineWidth', 4);
title('\fontsize{22} Joint Positions v.s. the Planned Positions')
xlabel('time [s]')
ylabel('position [deg]')
set(gca,'FontSize',16)
set(gca,'YColor','k')
hold on

h2 = plot(t,y(:,2).*180/pi, 'g-','LineWidth', 4);
set(gca,'FontSize',16)
hold on

yyaxis right
h3 = plot(t,y(:,3),'b-','LineWidth', 4);
ylabel('position [m]')
set(gca,'FontSize',16)
set(gca,'YColor','b')
hold on

yyaxis left
h11 = plot(0:dt:T,q(:,1).*180/pi, 'r:', 'LineWidth', 6);
hold on

h22 = plot(0:dt:T,q(:,2).*180/pi, 'g:','LineWidth', 6);
hold on

yyaxis right
h33 = plot(0:dt:T,q(:,3), 'b:','LineWidth',6);
legend([h1,h2,h3,h11,h22,h33],'\theta_1(dyn)','\theta_2(dyn)','d_3(dyn)','\theta_1(kine)','\theta_2(kine)','d_3(kine)')
grid
hold off


figure(5)

yyaxis left
h1 = plot(t,y(:,4).*180/pi, 'r-','LineWidth', 4);
title('\fontsize{22} Joint Velocities v.s. the Planned Velocities')
xlabel('time [s]')
ylabel('velocity [deg/s]')
set(gca,'FontSize',16)
set(gca,'YColor','k')
hold on

h2 = plot(t,y(:,5).*180/pi, 'g-','LineWidth', 4);
set(gca,'FontSize',16)
hold on

yyaxis right
h3 = plot(t,y(:,6),'b-','LineWidth', 4);
ylabel('velocity [m/s]')
set(gca,'FontSize',16)
set(gca,'YColor','b')
hold on

yyaxis left
h11 = plot(0:dt:T,q_dot(:,1).*180/pi, 'r:', 'LineWidth', 6);
hold on

h22 = plot(0:dt:T,q_dot(:,2).*180/pi, 'g:','LineWidth', 6);
hold on

yyaxis right
h33 = plot(0:dt:T,q_dot(:,3), 'b:','LineWidth',6);
legend([h1,h2,h3,h11,h22,h33],'$\dot{\theta_1}$(dyn)','$\dot{\theta_2}$(dyn)'...
    ,'$\dot{d_3}$(dyn)','$\dot{\theta_1}$(kine)','$\dot{\theta_2}$(kine)',...
    '$\dot{d_3}$(kine)','Interpreter','latex')
grid
hold off


xd=x_plan('pol',t);
x=forward_kin(y(:,1:3));

PathLength=((xA-xB)^2+(yA-yB)^2+(zA-zB)^2)^0.5;
err=100*(((xd(1,:)-x(1,:)).^2+(xd(2,:)-x(2,:)).^2+(xd(3,:)-x(3,:)).^2).^0.5)/PathLength;

figure(6)
plot(t,err,'LineWidth',4);
title('\fontsize{22} The Error Norm for the Position of the End-Effector ')
xlabel('time [s]')
ylabel('Error [%]')
grid
set(gca,'FontSize',16)

%% 5 new initial position

T=2; dt=0.001; t=0:dt:T;

zA=0.51;

q=q_plan('pol',t);
q_dot=q_dot_plan('pol',t,'diff');
tspan=0:0.0001:2;
y0=[q(1,:) 0 0 0];
option=odeset('RelTol',1e-4,'AbsTol',1e-8);
[t,y]=ode45(@state_eq,tspan,y0,option);

figure(7)

yyaxis left
h1 = plot(t,y(:,1).*180/pi, 'r-','LineWidth', 4);
title('\fontsize{22} Joint Positions v.s. the Planned Positions')
xlabel('time [s]')
ylabel('position [deg]')
set(gca,'FontSize',16)
set(gca,'YColor','k')
hold on

h2 = plot(t,y(:,2).*180/pi, 'g-','LineWidth', 4);
set(gca,'FontSize',16)
hold on

yyaxis right
h3 = plot(t,y(:,3),'b-','LineWidth', 4);
ylabel('position [m]')
set(gca,'FontSize',16)
set(gca,'YColor','b')
hold on

yyaxis left
h11 = plot(0:dt:T,q(:,1).*180/pi, 'r:', 'LineWidth', 6);
hold on

h22 = plot(0:dt:T,q(:,2).*180/pi, 'g:','LineWidth', 6);
hold on

yyaxis right
h33 = plot(0:dt:T,q(:,3), 'b:','LineWidth',6);
legend([h1,h2,h3,h11,h22,h33],'\theta_1(dyn)','\theta_2(dyn)','d_3(dyn)','\theta_1(kine)','\theta_2(kine)','d_3(kine)')
grid
hold off


figure(8)

yyaxis left
h1 = plot(t,y(:,4).*180/pi, 'r-','LineWidth', 4);
title('\fontsize{22} Joint Velocities v.s. the Planned Velocities')
xlabel('time [s]')
ylabel('velocity [deg/s]')
set(gca,'FontSize',16)
set(gca,'YColor','k')
hold on

h2 = plot(t,y(:,5).*180/pi, 'g-','LineWidth', 4);
set(gca,'FontSize',16)
hold on

yyaxis right
h3 = plot(t,y(:,6),'b-','LineWidth', 4);
ylabel('velocity [m/s]')
set(gca,'FontSize',16)
set(gca,'YColor','b')
hold on

yyaxis left
h11 = plot(0:dt:T,q_dot(:,1).*180/pi, 'r:', 'LineWidth', 6);
hold on

h22 = plot(0:dt:T,q_dot(:,2).*180/pi, 'g:','LineWidth', 6);
hold on

yyaxis right
h33 = plot(0:dt:T,q_dot(:,3), 'b:','LineWidth',6);
legend([h1,h2,h3,h11,h22,h33],'$\dot{\theta_1}$(dyn)','$\dot{\theta_2}$(dyn)'...
    ,'$\dot{d_3}$(dyn)','$\dot{\theta_1}$(kine)','$\dot{\theta_2}$(kine)',...
    '$\dot{d_3}$(kine)','Interpreter','latex')
grid
hold off


xd=x_plan('pol',t);
x=forward_kin(y(:,1:3));

PathLength=((xA-xB)^2+(yA-yB)^2+(zA-zB)^2)^0.5;
err=100*(((xd(1,:)-x(1,:)).^2+(xd(2,:)-x(2,:)).^2+(xd(3,:)-x(3,:)).^2).^0.5)/PathLength;

figure(9)
plot(t,err,'LineWidth',4);
title('\fontsize{22} The Error Norm for the Position of the End-Effector ')
xlabel('time [s]')
ylabel('Error [%]')
grid
set(gca,'FontSize',16)


%% 6 new load mass

T=2; dt=0.001; t=0:dt:T;

M = 0.6;

q=q_plan('pol',t);
q_dot=q_dot_plan('pol',t,'diff');
tspan=0:0.0001:2;
y0=[q(1,:) 0 0 0];
option=odeset('RelTol',1e-4,'AbsTol',1e-8);
[t,y]=ode45(@state_eq,tspan,y0,option);

figure(10)

yyaxis left
h1 = plot(t,y(:,1).*180/pi, 'r-','LineWidth', 4);
title('\fontsize{22} Joint Positions v.s. the Planned Positions')
xlabel('time [s]')
ylabel('position [deg]')
set(gca,'FontSize',16)
set(gca,'YColor','k')
hold on

h2 = plot(t,y(:,2).*180/pi, 'g-','LineWidth', 4);
set(gca,'FontSize',16)
hold on

yyaxis right
h3 = plot(t,y(:,3),'b-','LineWidth', 4);
ylabel('position [m]')
set(gca,'FontSize',16)
set(gca,'YColor','b')
hold on

yyaxis left
h11 = plot(0:dt:T,q(:,1).*180/pi, 'r:', 'LineWidth', 6);
hold on

h22 = plot(0:dt:T,q(:,2).*180/pi, 'g:','LineWidth', 6);
hold on

yyaxis right
h33 = plot(0:dt:T,q(:,3), 'b:','LineWidth',6);
legend([h1,h2,h3,h11,h22,h33],'\theta_1(dyn)','\theta_2(dyn)','d_3(dyn)','\theta_1(kine)','\theta_2(kine)','d_3(kine)')
grid
hold off


figure(11)

yyaxis left
h1 = plot(t,y(:,4).*180/pi, 'r-','LineWidth', 4);
title('\fontsize{22} Joint Velocities v.s. the Planned Velocities')
xlabel('time [s]')
ylabel('velocity [deg/s]')
set(gca,'FontSize',16)
set(gca,'YColor','k')
hold on

h2 = plot(t,y(:,5).*180/pi, 'g-','LineWidth', 4);
set(gca,'FontSize',16)
hold on

yyaxis right
h3 = plot(t,y(:,6),'b-','LineWidth', 4);
ylabel('velocity [m/s]')
set(gca,'FontSize',16)
set(gca,'YColor','b')
hold on

yyaxis left
h11 = plot(0:dt:T,q_dot(:,1).*180/pi, 'r:', 'LineWidth', 6);
hold on

h22 = plot(0:dt:T,q_dot(:,2).*180/pi, 'g:','LineWidth', 6);
hold on

yyaxis right
h33 = plot(0:dt:T,q_dot(:,3), 'b:','LineWidth',6);
legend([h1,h2,h3,h11,h22,h33],'$\dot{\theta_1}$(dyn)','$\dot{\theta_2}$(dyn)'...
    ,'$\dot{d_3}$(dyn)','$\dot{\theta_1}$(kine)','$\dot{\theta_2}$(kine)',...
    '$\dot{d_3}$(kine)','Interpreter','latex')
grid
hold off


xd=x_plan('pol',t);
x=forward_kin(y(:,1:3));

PathLength=((xA-xB)^2+(yA-yB)^2+(zA-zB)^2)^0.5;
err=100*(((xd(1,:)-x(1,:)).^2+(xd(2,:)-x(2,:)).^2+(xd(3,:)-x(3,:)).^2).^0.5)/PathLength;

figure(12)
plot(t,err,'LineWidth',4);
title('\fontsize{22} The Error Norm for the Position of the End-Effector ')
xlabel('time [s]')
ylabel('Error [%]')
grid
set(gca,'FontSize',16)

%% The following part is section 1, 3

syms th1 th1d th1dd th2 th2d th2dd d3 d3d d3dd l1 l2 l l4 m1 m2 m3 M g t Fex Fey Fez

q = [th1 th2 d3];
qd = [th1d th2d d3d];
qdd = [th1dd th2dd d3dd];

A01 = [cos(q(1)) -sin(q(1)) 0 0; sin(q(1)) cos(q(1)) 0 0; 0 0 1 l1; 0 0 0 1];
A12 = [cos(q(2)) 0 -sin(q(2)) 0; 0 1 0 0; sin(q(2)) 0 cos(q(2)) 0; 0 0 0 1];
A23 = [1 0 0 l2; 0 1 0 0; 0 0 1 q(3); 0 0 0 1];
A3t = [1 0 0 0; 0 1 0 0; 0 0 1 l4; 0 0 0 1];
A02 = A01*A12;
A03 = A02*A23;
A0t = A03*A3t;

R12 = A12(1:3,1:3);
R23 = A23(1:3,1:3);
f = [A0t(1,4); A0t(2,4); A0t(3,4)];

u1 = [0; 0; 1];
r1 = f;
J1 = [cross(u1, r1);u1];

R02 = A01(1:3,1:3)*A12(1:3,1:3);
u2 = R02*[0; -1; 0];
r2 = f - [0; 0; l1];
J2 = simplify([cross(u2, r2);u2]);

R0t = A01(1:3,1:3)*A12(1:3,1:3)*A23(1:3,1:3)*A3t(1:3,1:3);
u3 = R0t*[0; 0; 1];
J3 = [u3;0; 0; 0];

J = [J1 J2 J3];
Jwl = J(1:3, 1:3);
Jwa = J(4:6, 1:3);

I1 = [m1*l1^2/12 0 0; 0 m1*l1^2/12 0; 0 0 0];
I2 = [0 0 0; 0 m2*l2^2/12 0; 0 0 m2*l2^2/12];
I3 = [m3*l^2/12 0 0; 0 m3*l^2/12 0; 0 0 0];

%% 1a Lagrange

rcm1 = [0 0 l1/2].';
vcm1 = [gradient(rcm1(1), t) gradient(rcm1(2), t) gradient(rcm1(3), t)].';
w1 = [0 0 th1d].';
U1 = m1*g*l1/2;

rcm2 = [l2/2*cos(th2)*cos(th1) l2/2*cos(th2)*sin(th1) l1+l2/2*sin(th2)].';
vcm2 = [qd*gradient(rcm2(1), q) qd*gradient(rcm2(2), q) qd*gradient(rcm2(3), q)].';
w2 = R12.'*w1 + [0 -th2d 0].';
U2 = m2*g*(l1+l2*sin(th2)/2);

rcm3 = [l2*cos(th2)*cos(th1)-(d3+l4-l/2)*sin(th2)*cos(th1) l2*cos(th2)*sin(th1)-(d3+l4-l/2)*sin(th2)*sin(th1) l1++l2*sin(th2)+(d3+l4-l/2)*cos(th2)].';
vcm3 = [qd*gradient(rcm3(1), q) qd*gradient(rcm3(2), q) qd*gradient(rcm3(3), q)].';
w3 = R23.'*w2;
U3 = m3*g*(l1+l2*sin(th2)+(d3+l4-l/2)*cos(th2));

rcM = [l2*cos(th2)*cos(th1)-(d3+l4)*sin(th2)*cos(th1) l2*cos(th2)*sin(th1)-(d3+l4)*sin(th2)*sin(th1) l1++l2*sin(th2)+(d3+l4)*cos(th2)].';
vcM = [qd*gradient(rcM(1), q) qd*gradient(rcM(2), q) qd*gradient(rcM(3), q)].';
UM = M*g*(l1+l2*sin(th2)+(d3+l4)*cos(th2));

T = 1/2*(m1*vcm1.'*vcm1+w1.'*I1*w1+m2*vcm2.'*vcm2+w2.'*I2*w2+m3*vcm3.'*vcm3+w3.'*I3*w3+M*vcM.'*vcM);
U = U1 + U2 + U3 + UM;
L = T - U;

Fe = [Fex Fey Fez].';
rFe = [l2*cos(th2)*cos(th1)-(d3+l4)*sin(th2)*cos(th1) l2*cos(th2)*sin(th1)-(d3+l4)*sin(th2)*sin(th1) l1++l2*sin(th2)+(d3+l4)*cos(th2)].';

dL_dqd = gradient(L,qd);
ddt_dL_dqd = jacobian(dL_dqd,q)*qd.'+jacobian(dL_dqd,qd)*qdd.';
dL_dq = gradient(L,q);

drFe_dq = jacobian(rFe,q);
Q = (Fe.'*drFe_dq).';

eqa = ddt_dL_dqd - dL_dq - Q;

Ha = simplify(hessian(L,qd));

Ga = simplify(jacobian(U,q)).';

%% 1b Jacobian

Jtl = simplify(A0t(1:3,1:3).'*J(1:3,1:3));
Jta = simplify(A0t(1:3,1:3).'*J(4:6,1:3));

Jtl1 = subs(Jtl, [l4 d3 l2 th2], [0 0 0 0]);
Jtl1(:,2:3) = zeros;
Jta1 = subs(Jta, [l4 d3 l2 th2], [0 0 0 0]);
Jta1(:,2:3) = zeros;

Jtl2 = subs(Jtl, [l4 d3 l2], [0 0 l2/2]);
Jtl2(:,3) = zeros;
Jta2 = subs(Jta, [l4 d3 l2], [0 0 l2/2]);
Jta2(:,3) = zeros;

Jtl3 = subs(Jtl, [l4 d3], [0 d3+l4-l/2]);
Jta3 = subs(Jta, [l4 d3], [0 d3+l4-l/2]);

Jwl1 = subs(Jwl, [l4 d3 l2 th2], [0 0 0 0]);
Jwl1(:,2:3) = zeros;
Jwa1 = subs(Jwa, [l4 d3 l2 th2], [0 0 0 0]);
Jwa1(:,2:3) = zeros;

Jwl2 = subs(Jwl, [l4 d3 l2], [0 0 l2/2]);
Jwl2(:,3) = zeros;
Jwa2 = subs(Jwa, [l4 d3 l2], [0 0 l2/2]);
Jwa2(:,3) = zeros;

Jwl3 = subs(Jwl, [l4 d3], [0 d3+l4-l/2]);
Jwa3 = subs(Jwa, [l4 d3], [0 d3+l4-l/2]);

Hb = simplify(m1*Jtl1.'*Jtl1 + m2*Jtl2.'*Jtl2 + m3*Jtl3.'*Jtl3 + Jta1.'*I1*Jta1 + Jta2.'*I2*Jta2 + Jta3.'*I3*Jta3 + M*Jtl.'*Jtl);

dH11dq2 = gradient(Hb(1,1),th2);
dH11dq3 = gradient(Hb(1,1),d3);
dH22dq3 = gradient(Hb(2,2),d3);

C11 = 0.5*(dH11dq2*qd(2)+dH11dq3*qd(3));
C12 = 0.5*(dH11dq2*qd(1));
C13 = 0.5*(dH11dq3*qd(1));
C21 = 0.5*(-dH11dq2*qd(1));
C22 = 0.5*(dH22dq3*qd(3));
C23 = 0.5*(dH22dq3*qd(2));
C31 = 0.5*(-dH11dq3*qd(1));
C32 = 0.5*(-dH22dq3*qd(2));
C33 = 0;

C = [C11 C12 C13;
    C21 C22 C23;
    C31 C32 0];

Gb = simplify(-(m1*Jwl1.'+m2*Jwl2.'+m3*Jwl3.'+M*Jwl.')*[0;0;-g]);

JwlTFe = Jwl.'*Fe;

eqb = Hb*qdd.' + C*qd.' + Gb - JwlTFe;


%% compare 1a 1b

compare_H = simplify(Hb - Ha)
compare_G = simplify(Gb - Ga)
compare_eq1 = simplify(eqb(1) - eqa(1))
compare_eq2 = simplify(eqb(2) - eqa(2))
compare_eq3 = simplify(eqb(3) - eqa(3))

%% 3 Reaction force
syms th1 th2 d3 th1d th2d d3d th1dd th2dd d3dd

q = [th1 th2 d3];
qd = [th1d th2d d3d];
qdd = [th1dd th2dd d3dd];

w0 = [0;0;0];
an0 = [0;0;0];
ac0 = [0;0;0];
ae0 = [0;0;0];

% Note that "an" means "angular acceleration", which denoted by "alpha" in report.
% "a" simply means "acceleration", which denoted by "a" in report.
% Don't confuse!

% joint 1
r1c1=[0;0;l1/2];
r1e1=[0;0;l1];
R01 = [cos(q(1)) -sin(q(1)) 0; sin(q(1)) cos(q(1)) 0; 0 0 1];
u1=[0;0;1];

w1=R01.'*w0+u1*qd(1);
an1=R01.'*an0+u1*qdd(1)+cross(w1,u1)*qd(1);
ac1=R01.'*ae0+cross(an1,r1c1)+cross(w1,cross(w1,r1c1));
ae1=R01.'*ae0+cross(an1,r1e1)+cross(w1,cross(w1,r1e1));

% joint 2
r2c2=[l2/2;0;0];
r2e2=[l2;0;0];
R12 = [cos(q(2)) 0 -sin(q(2)); 0 1 0; sin(q(2)) 0 cos(q(2))];
u2=[0;-1;0];

w2=R12.'*w1+u2*qd(2);
an2=simplify(R12.'*an1+u2*qdd(2)+cross(w2,u2)*qd(2));
ac2=simplify(R12.'*ae1+cross(an2,r2c2)+cross(w2,cross(w2,r2c2)));
ae2=simplify(R12.'*ae1+cross(an2,r2e2)+cross(w2,cross(w2,r2e2)));

% joint 3
r3c3=[0;0;d3+l4-l/2];
r3e3=[0;0;d3+l4];
R23=[1 0 0; 0 1 0; 0 0 1];
u3=[0;0;1];

w3=R23.'*w2;
an3=simplify(R23.'*an2);
ac3=simplify(R23.'*ae2+cross(an3,r3c3)+cross(w3,cross(w3,r3c3))+u3*qdd(3)+2*(cross(w3,u3)*qd(3)));
ae3=simplify(R23.'*ae2+cross(an3,r3e3)+cross(w3,cross(w3,r3e3))+u3*qdd(3)+2*(cross(w3,u3)*qd(3)));

% force on M
R03 = A03(1:3,1:3);
fM = simplify(M*ae3 - M*R03.'*[0;0;-g]);

% force on link 3
R3t = A3t(1:3,1:3);
f3 = simplify(m3*ac3 + R3t*fM - m3*R03.'*[0;0;-g]);

% force on link 2
R02 = A02(1:3,1:3);
R23 = A23(1:3,1:3);
f2 = simplify(m2*ac2 + R23*f3 - m2*R02.'*[0;0;-g]);