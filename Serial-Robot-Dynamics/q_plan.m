function q=q_plan(prof,t)

global elbows

x = x_plan(prof,t);

q = inverse_kin(x,elbows);

q(1,:) = q(2,:) - 0.5*(q(3,:) - q(2,:));

% Check the mechanical limitations
for i = 1: length(t)
    if q(i,1)<-pi || q(i,1)>pi
        fprintf('t = %f, theta_1 = %f => fail\n', t(i), q(i,1));
        break
    elseif q(i,2)<-pi || q(i,2)>pi
        fprintf('t = %f, theta_2 = %f => fail\n', t(i), q(i,2));
        break
    elseif abs(q(i,3))>=0.7
        fprintf('t = %f, d_3 = %f => fail\n', t(i), q(i,3));
        break
    end
end

end