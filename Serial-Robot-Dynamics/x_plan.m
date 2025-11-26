function x = x_plan(prof, t)

% prof – decision value for the different velocity profile.
% ('con': Constant profile)
% ('tra': Trapezoidal profile)
% ('pol': Polynomial profile)
% x – the position of the tool's origin in time t. 3(xyz)*i(time)
% t - a time row vector, 1*i(time)

global xA yA zA xB yB zB T

switch prof
    case 'con'
        x = [xA + (xB-xA)/T*t; yA + (yB-yA)/T*t; zA + (zB-zA)/T*t];
    
    case 'tra'
        vc=[6*(xB-xA)/(5*T); 6*(yB-yA)/(5*T); 6*(zB-zA)/(5*T)];
        x = zeros(3, length(t));
        for i = 1:length(t)
            if t(1,i) < T/6
                x(:, i) = [xA+3*vc(1)*t(1,i).^2/T; yA+3*vc(2)*t(1,i).^2/T; zA+3*vc(3)*t(1,i).^2/T];
            elseif t(1,i) <= 5*T/6
                x(:, i) = [xA+vc(1)*(t(1,i)-T/12); yA+vc(2)*(t(1,i)-T/12); zA+vc(3)*(t(1,i)-T/12)];
            else
                x(:, i) = [xA+vc(1)*(t(1,i)-T/12)-3*vc(1)*(t(1,i)-5*T/6).^2/T; yA+vc(2)*(t(1,i)-T/12)-3*vc(2)*(t(1,i)-5*T/6).^2/T; zA+vc(3)*(t(1,i)-T/12)-3*vc(3)*(t(1,i)-5*T/6).^2/T];
            end
        end
    
    case 'pol'
        x = zeros(3, length(t));
        for j = 1:length(t)
            x(:, j) = [6*(xB-xA)*t(j)^5/T^5-15*(xB-xA)*t(j)^4/T^4+10*(xB-xA)*t(j)^3/T^3+xA;
                       6*(yB-yA)*t(j)^5/T^5-15*(yB-yA)*t(j)^4/T^4+10*(yB-yA)*t(j)^3/T^3+yA;
                       6*(zB-zA)*t(j)^5/T^5-15*(zB-zA)*t(j)^4/T^4+10*(zB-zA)*t(j)^3/T^3+zA];
        end

end
end