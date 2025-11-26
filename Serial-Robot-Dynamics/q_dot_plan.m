function q_dot = q_dot_plan(prof,t,method)

% prof – decision value for the different velocity profile.
% ('con': Constant profile)
% ('tra': Trapezoidal profile)
% ('pol': Polynomial profile)
% q_dot – the joints velocity in time t, i(time)*6(joints)
% t - a time row vector, 1*i(time)
% method = 'diff': Numeric differentiation of the joint positions 𝒒(𝑡).
% method = 'rela': From the relation 𝒗 = 𝐽𝒒̇

global dt

q = q_plan(prof,t);
v = v_plan(prof,t);

switch method
    case 'diff'
        qdot = diff(q)/dt;
        q_dot = [0 0 0; qdot];
    case 'rela'
        for i = 1:length(t)
            J = jacobian_mat(q(i,:));
            invJ_L=inv(J(1:3,1:3));
            q_dot(i,:)=invJ_L*v(:,i);
        end
end

end