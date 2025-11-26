function Xdot=state_eq(t,X)

global dt controller M z1  int_qd_t q_dot2 P qd qd_dot qd_dot2 qd_t qd_dot_t qd_dot2_t a tau_t P_dot qdot2



q=X(1:3);
q_dot=X(4:6);
z1=X(1:3);
z2=X(4:6);

t_vec=0:dt:((length(qd)-1)*dt);

qd_t=zeros(1,3);
qd_dot_t=zeros(1,3);
qd_dot2_t=zeros(1,3);

qd_t(1)=(interp1(t_vec,qd(:,1)',t))';
qd_t(2)=(interp1(t_vec,qd(:,2)',t))';
qd_t(3)=(interp1(t_vec,qd(:,3)',t))';

qd_dot_t(1)=(interp1(t_vec,qd_dot(:,1)',t))';
qd_dot_t(2)=(interp1(t_vec,qd_dot(:,2)',t))';
qd_dot_t(3)=(interp1(t_vec,qd_dot(:,3)',t))';

qd_dot2_t(1)=(interp1(t_vec,qd_dot2(:,1)',t))';
qd_dot2_t(2)=(interp1(t_vec,qd_dot2(:,2)',t))';
qd_dot2_t(3)=(interp1(t_vec,qd_dot2(:,3)',t))';

[H,C,G]=dynamics_mat(z1,z2,M);

if strcmp(controller,'PDINV')
    % Inverse dynamics + PD
    tau_t=tau_cont(z1,z2,t,controller);
    Xdot=[z2;H\(tau_t-C*z2-G);P_dot;tau_t];
end
if strcmp(controller,'PDG')
    % PD + gravity compensation
    tau_t=tau_cont(z1,z2,t,controller);
    Xdot=[z2;
        H\(tau_t-C*z2-G);P_dot;tau_t];
end

if strcmp(controller,'PID')
%     z2=X(4:6);
    z3=X(7:9);
    int_qd_t=X(14:16);

    [H,C,G]=dynamics_mat(z2,z3,M);
    
    tau_t=tau_cont(z2,z3,t,controller);

    Xdot=[z2;z3;H\(tau_t-C*z3-G);P_dot;tau_t;qd_t'];
end
if strcmp(controller,'MINMAX')
    tau_t=tau_cont(q,q_dot,t,controller);
    Xdot=[z2;H\(tau_t-C*z2-G);P_dot;tau_t];
end
if strcmp(controller,'AC')
    P=X(7);
    q=z1;qdot=z2;
    %e=q-qd_t;
    %edot=qdot-q_dotd_t;
    qdot2=H\(tau_t-C*z2-G);
    tau_t=tau_cont(q,qdot,t,controller);
    %P=P+Pdot*dt;
    Xdot=[z2;H\(tau_t-C*z2-G);P_dot;tau_t];
end


% else
% %     z1=q;
% %     z2=q_dot;
% %     [H,C,G]=dynamics_mat(z1,z2,M);
% 
%     tau_t=tau_cont(q,q_dot,t,controller);
% 
%     Xdot=[z2; H\(tau_t-C*z2-G);P_dot;tau_t];
% 
%     if strcmp(controller,'AC')
%         q_dot2=H\(tau_t-C*z2-G);
%         P=X(7);
%     end
% 
% end

