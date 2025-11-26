function v = v_plan(prof, t)

% prof – decision value for the different velocity profile.
% ('con': Constant profile)
% ('tra': Trapezoidal profile)
% ('pol': Polynomial profile)
% v – the velocity of the tool,s origin in time t, 3(xyz)*i(time)
% t - a time row vector, 1*i(time)

global xA yA zA xB yB zB T

switch prof
    case 'con'
        v = zeros(3, length(t));
        for i = 1:length(t)
            v(:, i) = [(xB-xA)/T; (yB-yA)/T; (zB-zA)/T];
        end
    case 'tra'
        vc=[6*(xB-xA)/(5*T); 6*(yB-yA)/(5*T); 6*(zB-zA)/(5*T)];
        v = zeros(3, length(t));
        for j = 1:length(t)
            if t(1,j) < T/6
                v(:, j) = [6*vc(1)*t(1,j)/T; 6*vc(2)*t(1,j)/T; 6*vc(3)*t(1,j)/T];
            elseif t(1,j) <= 5*T/6
                v(:, j) = [vc(1), vc(2), vc(3)];
            else
                v(:, j) = [6*vc(1)-6*vc(1)*t(1,j)/T; 6*vc(2)-6*vc(2)*t(1,j)/T; 6*vc(3)-6*vc(3)*t(1,j)/T];
            end
        end
    
    case 'pol'
        v = zeros(3, length(t));
        for k = 1:length(t)
            v(:, k) = [(xB-xA)*(30/T^5*t(k)^4-60/T^4*t(k)^3+30/T^3*t(k)^2); (yB-yA)*(30/T^5*t(k)^4-60/T^4*t(k)^3+30/T^3*t(k)^2); (zB-zA)*(30/T^5*t(k)^4-60/T^4*t(k)^3+30/T^3*t(k)^2)];
        end
end

end