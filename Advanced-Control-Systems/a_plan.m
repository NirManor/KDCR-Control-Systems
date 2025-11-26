function a = a_plan(prof, t)

% prof – decision value for the different velocity profile.
% ('con': Constant profile)
% ('tra': Trapezoidal profile)
% ('pol': Polynomial profile)
% a – the acceleration of the tool’s origin in time t, 3(xyz)*i(time)
% t - a time row vector, 1*i(time)

global xA yA zA xB yB zB T

switch prof
    case 'con'
        a = zeros(3, length(t));
    
    case 'tra'
        vc=[6*(xB-xA)/(5*T); 6*(yB-yA)/(5*T); 6*(zB-zA)/(5*T)];
        a = zeros(3, length(t));
        for i = 1:length(t)
            if t(1,i) < T/6
                a(:, i) = [6*vc(1)/T; 6*vc(2)/T; 6*vc(3)/T];
            elseif t(1,i) <= 5*T/6
                a(:, i) = zeros(3,1);
            else
                a(:, i) = [-6*vc(1)/T; -6*vc(2)/T; -6*vc(3)/T];
            end
        end
    
    case 'pol'
        a = zeros(3, length(t));
        for j = 1:length(t)
            a(:, j) = [(xB-xA)*(120/T^5*t(1, j)^3-180/T^4*t(1, j)^2+60/T^3*t(1, j)); (yB-yA)*(120/T^5*t(1, j)^3-180/T^4*t(1, j)^2+60/T^3*t(1, j)); (zB-zA)*(120/T^5*t(1, j)^3-180/T^4*t(1, j)^2+60/T^3*t(1, j))];
        end

end
end