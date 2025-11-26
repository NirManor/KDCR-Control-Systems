% function [H,C,G]=dynamics_mat_test(q,q_dot)
% global 
clear;clc;close all
syms M m1 m2 m3 l2 g l teta1 teta2 teta1_dot teta2_dot d3_dot d3 l4 Mmax C2 S2
H11=M*(cos(teta2)*l2-sin(teta2)*(d3+l4))^2+m3*(cos(teta2)*l2-(sin(teta2)*(d3+l4-0.5*l)))^2+(cos(teta2)^2*l2^2*m2)/3+...
    (sin(teta2)^2*l^2*m3)/12;
H22=M*(d3+l4)^2 + M*l2^2 + (l^2*m3)/12 + (l2^2*m2)/3 + l2^2*m3+m3*(d3+l4-0.5*l)^2;
H23=l2*(M + m3);
H33=M + m3;
H=[H11 0 0;
    0 H22 H23;
    0 H23 H33];
H=simplify(H)


H11_q2=diff(H11,teta2);
H11_q3=diff(H11,d3);
H22_q3=diff(H22,d3);

C=0.5*[H11_q2*teta2_dot+H11_q3*d3_dot H11_q2*teta1_dot H11_q3*teta1_dot;
    -1*H11_q2*teta1_dot H22_q3*d3_dot H22_q3*teta2_dot;
    -1*H11_q3*teta1_dot -1*H22_q3*teta2_dot 0];

C=simplify(C)

G1=0;
G2=g*(M*(C2*l2 - S2*(d3+l4)) + m3*(C2*l2 - S2*(d3+l4-0.5*l)) + (C2*l2*m2)/2);
G3=C2*g*(M + m3);
G=[G1;G2;G3];

G=simplify(G)


%% M=0


H_0=subs(H,M,0);

H_0=subs(H_0,cos(teta2),C2);
H_0=simplify(subs(H_0,sin(teta2),S2));
H_0



C_0=C;

C_0=subs(C_0,cos(teta2),C2);
C_0=(subs(C_0,sin(teta2),S2));
C_0=subs(C_0,M,0);
C_0=simplify(C_0)


G_0=subs(G,M,0)




%% m2,m3=0

H_m0=subs(H,M,Mmax);



H_m0=subs(H_m0,cos(teta2),C2);
H_m0=subs(H_m0,sin(teta2),S2);
H_m0=subs(H_m0,m2,0);
H_m0=subs(H_m0,m3,0);

H_m0

C_m0=C;
C_m0=subs(C_m0,cos(teta2),C2);
C_m0=subs(C_m0,sin(teta2),S2);
C_m0=subs(C_m0,M,Mmax);
C_m0=subs(C_m0,m2,0);
C_m0=subs(C_m0,m3,0);

C_m0=simplify(C_m0)

G_m0=subs(G,M,Mmax);
G_m0=subs(G_m0,m2,0);
G_m0=subs(G_m0,m3,0);

G_m0


