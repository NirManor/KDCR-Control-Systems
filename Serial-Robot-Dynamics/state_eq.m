function Xdot=state_eq(t,X)

global tau dt T

q=X(1:3);
q_dot=X(4:6);
z1(1)=q(1);
z1(2)=q(2);
z1(3)=q(3);
z2(1)=q_dot(1);
z2(2)=q_dot(2);
z2(3)=q_dot(3);

t_vec=0:dt:T;
tau_t=(interp1(t_vec,tau',t))';

[H,C,G]=dynamics_mat(z1,z2);
Xdot=[z2';
    inv(H)*(tau_t-C*z2'-G)];
