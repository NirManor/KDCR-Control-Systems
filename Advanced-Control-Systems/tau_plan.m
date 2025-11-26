function tau=tau_plan(prof,t)

global M

tau=zeros(3,length(t));
q=q_plan(prof,t);
q_dot=q_dot_plan(prof,t,'diff');
q_dot2=q_dot2_plan(prof,t,'diff');

for i=1:length(t)
    [H,C,G]=dynamics_mat(q(i,:),q_dot(i,:),M);
    tau(:,i)=H*q_dot2(i,:)'+C*q_dot(i,:)'+G;
end

end