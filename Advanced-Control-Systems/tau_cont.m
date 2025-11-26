function tau=tau_cont(q,q_dot,t,law)

global m2 m3 l2 l l4 g Kp Ki Kd gama F q_dotd_t P qd_t qd_dot_t qd_dot2_t z1 int_qd_t K Beta delta P_dot qdot2

%        m2 m3 l2 g l Kp Ki Kd gama F qdot2 P qd_t q_dotd_t q_dot2d_t z1 int_qd_t K Beta delta qd_dot2_t qd_dot_t

P_dot = 1;

switch law
    case 'PDINV'
        z1=q;
        z2=q_dot;
        % Since the control laws are not aware of the change of mass,
        % M still is 0.5 kg
        [H,C,G]=dynamics_mat(z1,z2,0.5);
        tau=C*z2+G+H*(qd_dot2_t'-Kp*(z1-qd_t')-Kd*(z2-qd_dot_t'));
    case 'PDG'
        z1=q;
        z2=q_dot;
        % Since the control laws are not aware of the change of mass,
        % M still is 0.5 kg
        [~,~,G]=dynamics_mat(z1,z2,0.5);
        tau=G-Kp*(z1-qd_t.')-Kd*(z2-qd_dot_t.');
    case 'PID'
        z2=q;
        z3=q_dot;
        tau=-Kd*(z3-qd_dot_t')-Kp*(z2-qd_t')-Ki*(z1-int_qd_t);   
    case 'MINMAX'
        e=q-qd_t';
        e_dot=q_dot-qd_dot_t.';
        s=e_dot+K*e;

        qrdot=qd_dot_t.'-K*e;
        qrdot2=qd_dot2_t.'-K*e_dot;

        teta2=q(2);
        d3=q(3);
        
        teta1_dot=q_dot(1);
        teta2_dot=q_dot(2);
        d3_dot=q_dot(3);
        C2=cos(teta2);
        S2=sin(teta2);

        %[H0,C0,G0]=dynamics_mat(q,qdot,0);

        H0_11=m3*(C2*l2 - S2*(d3 - l/2 + l4))^2 + (C2^2*l2^2*m2)/3 + (S2^2*l^2*m3)/12;
        H0_22=m3*(d3 - l/2 + l4)^2 + (l^2*m3)/12 + (l2^2*m2)/3 + l2^2*m3;
        H0_23=l2*m3;
        H0_33=m3;
        H0=[H0_11 0 0;
            0 H0_22 H0_23;
            0 H0_23 H0_33];
        C0_11=- (teta2_dot*(2*m3*(S2*l2 + C2*(d3 - l/2 + l4))*(C2*l2 - S2*(d3 - l/2 + l4)) - (C2*S2*l^2*m3)/6 + (2*C2*S2*l2^2*m2)/3))/2 - S2*d3_dot*m3*(C2*l2 - S2*(d3 - l/2 + l4));
        C0_12=-(teta1_dot*(2*m3*(S2*l2 + C2*(d3 - l/2 + l4))*(C2*l2 - S2*(d3 - l/2 + l4)) - (C2*S2*l^2*m3)/6 + (2*C2*S2*l2^2*m2)/3))/2;
        C0_13=-S2*m3*teta1_dot*(C2*l2 - S2*(d3 - l/2 + l4));
        C0_21=-C0_12;
        C0_22=(d3_dot*m3*(2*d3 - l + 2*l4))/2;
        C0_23=(m3*teta2_dot*(2*d3 - l + 2*l4))/2;
        C0_31=-C0_13;
        C0_32=-C0_23;
        C0_33=0;
        C0=[C0_11 C0_12 C0_13;
            C0_21 C0_22 C0_23;
            C0_31 C0_32 C0_33];
        G0_1=0;
        G0_2=g*(m3*(C2*l2 - S2*(d3 - l/2 + l4)) + (C2*l2*m2)/2);
        G0_3=C2*g*m3;
        G0=[G0_1;G0_2;G0_3];

        eta0=-H0*qrdot2-C0*qrdot-G0+P*e;
        
        %m2,m3=0

        Mmax=1;
        Htilda_11=Mmax*(C2*l2 - S2*(d3 + l4))^2;
        Htilda_22=Mmax*(d3 + l4)^2 + Mmax*l2^2;
        Htilda_23=l2*Mmax;
        Htilda_33=Mmax;
        Htilda=[Htilda_11 0 0;
            0 Htilda_22 Htilda_23;
            0 Htilda_23 Htilda_33];
        Ctilda_11=- Mmax*S2*d3_dot*(C2*l2 - S2*(d3 + l4)) - Mmax*teta2_dot*(S2*l2 + C2*(d3 + l4))*(C2*l2 - S2*(d3 + l4));
        Ctilda_12=-Mmax*teta1_dot*(S2*l2 + C2*(d3 + l4))*(C2*l2 - S2*(d3 + l4));
        Ctilda_13=-Mmax*S2*teta1_dot*(C2*l2 - S2*(d3 + l4));
        Ctilda_21=Mmax*teta1_dot*(S2*l2 + C2*(d3 + l4))*(C2*l2 - S2*(d3 + l4));
        Ctilda_22= Mmax*d3_dot*(d3 + l4);
        Ctilda_23=Mmax*teta2_dot*(d3 + l4);
        Ctilda_31=Mmax*S2*teta1_dot*(C2*l2 - S2*(d3 + l4));
        Ctilda_32=-Mmax*teta2_dot*(d3 + l4);
        Ctilda_33=0;
        
        Ctilda=[Ctilda_11 Ctilda_12 Ctilda_13;
            Ctilda_21 Ctilda_22 Ctilda_23;
            Ctilda_31 Ctilda_32 Ctilda_33];
        
        Gtilda_1=0;
        Gtilda_2=Mmax*g*(C2*l2 - S2*(d3 + l4));
        Gtilda_3=C2*g*(Mmax);
        Gtilda=[Gtilda_1;Gtilda_2;Gtilda_3];


        eta_tilda=abs(norm(Htilda*qrdot2)+norm(Ctilda*qrdot)+norm(Gtilda));

        rou=max([0,s'*eta0/norm(s)+Beta*eta_tilda]);

        tau=-rou*s/(norm(s)+delta);

%        
%         
%         eta0=-H0*qrdot2-C0*qrdot-G0+P*e;
% 
% 
%         % Set  M = 0 to get eta0
%         [H0,C0,G0]=dynamics_mat(q,q_dot,0);
%         eta0=-H0*qr_dot2-C0*qr_dot-G0+P*e;
%         
%         % Set m2, m3 = 0 M = 1 to get eta_tilde
%         m2 = 0;
%         m3 = 0;
%         [H_tilde,C_tilde,G_tilde]=dynamics_mat(q,q_dot,1);
% 
%         eta_tilde=abs(norm(H_tilde*qr_dot2)+norm(C_tilde*qr_dot)+norm(G_tilde));
%         
%         if s.'*eta0/norm(s)+Beta*eta_tilde < 0
%             tau = 0;
%         else
%             rou=s.'*eta0/norm(s)+Beta*eta_tilde;
%             tau=-rou*s/(norm(s)+delta);
%         end
% 
%         % rou=max([0,s.'*eta0./norm(s)+Beta.*eta_tilde]);
%         % 
%         % tau=-rou.*s./(norm(s)+delta);
%         
%         % Set original m2, m3
%         m2 = m2 + 7800*l2*pi*0.015^2/4;
%         m3 = m3 + 7800*l*pi*0.015^2/4;

    case 'AC'
        % Set m2, m3 = 0 M = 1 to get eta_tilde

        teta2=q(2);
        d3=q(3);
        teta1_dot=q_dot(1);
        teta2_dot=q_dot(2);
        d3_dot=q_dot(3);
        C2=cos(teta2);
        S2=sin(teta2);

        % The following values are divided by M
        Mmax=1;
        Htilda_11=Mmax*(C2*l2 - S2*(d3 + l4))^2;
        Htilda_22=Mmax*(d3 + l4)^2 + Mmax*l2^2;
        Htilda_23=l2*Mmax;
        Htilda_33=Mmax;
        Htilda=[Htilda_11 0 0;
            0 Htilda_22 Htilda_23;
            0 Htilda_23 Htilda_33];
        Ctilda_11=- Mmax*S2*d3_dot*(C2*l2 - S2*(d3 + l4)) - Mmax*teta2_dot*(S2*l2 + C2*(d3 + l4))*(C2*l2 - S2*(d3 + l4));
        Ctilda_12=-Mmax*teta1_dot*(S2*l2 + C2*(d3 + l4))*(C2*l2 - S2*(d3 + l4));
        Ctilda_13=-Mmax*S2*teta1_dot*(C2*l2 - S2*(d3 + l4));
        Ctilda_21=Mmax*teta1_dot*(S2*l2 + C2*(d3 + l4))*(C2*l2 - S2*(d3 + l4));
        Ctilda_22= Mmax*d3_dot*(d3 + l4);
        Ctilda_23=Mmax*teta2_dot*(d3 + l4);
        Ctilda_31=Mmax*S2*teta1_dot*(C2*l2 - S2*(d3 + l4));
        Ctilda_32=-Mmax*teta2_dot*(d3 + l4);
        Ctilda_33=0;
        
        Ctilda=[Ctilda_11 Ctilda_12 Ctilda_13;
            Ctilda_21 Ctilda_22 Ctilda_23;
            Ctilda_31 Ctilda_32 Ctilda_33];
        
        Gtilda_1=0;
        Gtilda_2=Mmax*g*(C2*l2 - S2*(d3 + l4));
        Gtilda_3=C2*g*(Mmax);
        Gtilda=[Gtilda_1;Gtilda_2;Gtilda_3];

        
        htilda=Ctilda*q_dot+Gtilda;

        Y=(Htilda*qdot2+htilda);
        B=[zeros(3);eye(3)];
        [Hpre,Cpre,Gpre]=dynamics_mat(q,q_dot,P);
        e=q-qd_t';
        edot=q_dot-qd_dot_t';
        P_dot=-(gama^(-1))*Y'*inv(Hpre)'*B'*F'*[e;edot];
        tau=Hpre*(qd_dot2_t'-Kd*edot-Kp*e)+Cpre*q_dot+Gpre;



%         m2 = 0;
%         m3 = 0;
%         [H_tilde,C_tilde,G_tilde]=dynamics_mat(q,qdot,1);
% 
%         % Set original m2, m3
%         m2 = m2 + 7800*l2*pi*0.015^2/4;
%         m3 = m3 + 7800*l*pi*0.015^2/4;
% 
%         Y=(H_tilde*q_dot2+C_tilde*q_dot+G_tilde);
%         B=[zeros(3);eye(3)];
% 
%         [H_hat,C_hat,G_hat]=dynamics_mat(q,q_dot,P);
%         e=q-qd_t.';
%         e_dot=q_dot-qd_dot_t.';
% 
%         P_dot=-(gama^(-1))*Y'*inv(H_hat)'*B'*F'*[e;e_dot];
%         tau=H_hat*(qd_dot2_t.'-Kd*e_dot-Kp*e)+C_hat*q_dot+G_hat;
end

