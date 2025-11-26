function [H,C,G]=dynamics_mat(q,q_dot,M)
global m1 m2 m3 l1 l2 l l4 g

th1=q(1);
th2=q(2);
d3=q(3);

c2=cos(th2);
s2=sin(th2);

qd(1)=q_dot(1);
qd(2)=q_dot(2);
qd(3)=q_dot(3);

H11 = M*(d3*s2 - l2*c2 + l4*s2)^2 + m3*(s2*(d3 - l/2 + l4) - l2*c2)^2 + (l2^2*m2*c2^2)/3 + (l^2*m3*s2^2)/12;
H22 = m3*(d3 - l/2 + l4)^2 + M*(d3 + l4)^2 + M*l2^2 + (l2^2*m2)/3 + (l^2*m3)/12 + l2^2*m3;
H23 = l2*(M + m3);
H33 = M + m3;
H=[H11 0 0;
    0 H22 H23;
    0 H23 H33];

dH11dq2 = 2*m3*(c2*(d3 - l/2 + l4) + l2*s2)*(s2*(d3 - l/2 + l4) - l2*c2) + 2*M*(d3*c2 + l4*c2 + l2*s2)*(d3*s2 - l2*c2 + l4*s2) - (2*l2^2*m2*c2*s2)/3 + (l^2*m3*c2*s2)/6;
 
dH11dq3 = 2*M*s2*(d3*s2 - l2*c2 + l4*s2) + 2*m3*s2*(s2*(d3 - l/2 + l4) - l2*c2);
 
dH22dq3 = m3*(2*d3 - l + 2*l4) + M*(2*d3 + 2*l4);

C11 = 0.5*(dH11dq2*qd(2)+dH11dq3*qd(3));
C12 = 0.5*(dH11dq2*qd(1));
C13 = 0.5*(dH11dq3*qd(1));
C21 = 0.5*(-dH11dq2*qd(1));
C22 = 0.5*(dH22dq3*qd(3));
C23 = 0.5*(dH22dq3*qd(2));
C31 = 0.5*(-dH11dq3*qd(1));
C32 = 0.5*(-dH22dq3*qd(2));

C = [C11 C12 C13;
    C21 C22 C23;
    C31 C32 0];

G1 = 0;
G2 = -g*(M*(d3*s2 - l2*c2 + l4*s2) + m3*(s2*(d3 - l/2 + l4) - l2*c2) - (l2*m2*c2)/2);
G3 = g*c2*(M + m3);
G = [G1;G2;G3];