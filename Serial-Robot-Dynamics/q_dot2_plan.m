function q_dot2 = q_dot2_plan(prof,t,method)

% prof – decision value for the different velocity profile.
% ('con': Constant profile)
% ('tra': Trapezoidal profile)
% ('pol': Polynomial profile)
% q_dot2 – the second time derivative of the joints* parameters vector, i(time)*6(joints)
% t - a time row vector, 1*i(time)
% method = 'dif': Numeric differentiation of the joint positions 𝒒(𝑡).
% method = 'rel': From the relation 𝒗 = 𝐽*q_dot

global dt

q=q_plan(prof,t);
v=v_plan(prof,t);
a=a_plan(prof,t);
q_dot=q_dot_plan(prof,t,'diff');

switch method
    case 'diff'
        G=diff(q_dot)/dt;
        q_dot2=[0 0 0;G];
    case 'rela'
        for i=1:length(t)
            J = jacobian_mat(q(i,:));
            J_dot=jacobian_mat_dot(q(i,:),q_dot(i,:));
            invJ=inv(J(1:3,1:3));
            q_dot2(1:3,i)=invJ_L*(a(:,i)-J_dot(1:3,1:3)*q_dot(:,i));
        end
end
end