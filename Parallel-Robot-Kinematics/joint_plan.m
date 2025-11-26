function q=joint_plan(t)
% The following function calculates the position of the joints as a
% function of time and according the mechanical limitations. 
% t is vector of the time.

% Constants
global T dt R L r

x = pos_plan(t); % path planing in the tool frame
X=x(1,:);
Y=x(2,:);
phi=x(3,:);


for i=1:401
   
    x_a1(i)=X(i)+r*cos(phi(i)+pi/3);
    y_a1(i)=Y(i)+r*sin(phi(i)+pi/3);
    x_a2(i)=X(i);
    y_a2(i)=Y(i); 
    x_a3(i)=X(i)+r*cos(phi(i));
    y_a3(i)=Y(i)+r*sin(phi(i));
    

end

elbows=[1,1,1; -1,1,1; 1,-1,1 ;-1,-1,1];

for i=1:size(t,2)
    
    q1(i,:) = inverse_kin(x(:,i),elbows(1,:));
    q2(i,:) = inverse_kin(x(:,i),elbows(2,:));
    q3(i,:) = inverse_kin(x(:,i),elbows(3,:));
    q4(i,:) = inverse_kin(x(:,i),elbows(4,:));

end

Q3(1,:) = q1(:,3);
Q3(2,:) = q2(:,3);
Q3(3,:) = q3(:,3);
Q3(4,:) = q4(:,3);

d3 = zeros(1,size(t,2));
d3(1) = Q3(2,1);
for i=2:T/dt+1
    for j=2:2
        d3(i) = Q3(j,i);
    end
end

Q2(1,:) = q1(:,2);
Q2(2,:) = q2(:,2);
Q2(3,:) = q3(:,2);
Q2(4,:) = q4(:,2);

th2 = zeros(1,size(t,2));
th2(1) = Q2(2,1);
for i=2:T/dt+1
    for j=1:4

        [xi_23, yi_23] = polyxpoly([R*cos(Q2(j,i)) x_a2(i)], [R*sin(Q2(j,i)) y_a2(i)],[0 x_a3(i)]  , [-R   y_a3(i)] );

        if abs(th2(i-1)-Q2(j,i))<0.1 && abs((R*cos(Q2(j,i))-X(i))^2+(R*sin(Q2(j,i))-Y(i))^2 - L^2)<0.1 && isempty(xi_23)
            th2(i) = Q2(j,i);
            break
        end
    end
end

Q1(1,:) = q1(:,1);
Q1(2,:) = q2(:,1);
Q1(3,:) = q3(:,1);
Q1(4,:) = q4(:,1);

th1 = zeros(1,size(t,2));

th1(1) = Q1(2,1);
for i=2:T/dt+1
    for j=1:4
        
        [xi_13, yi_13] = polyxpoly([R*cos(Q1(j,i)) x_a1(i)], [R*sin(Q1(j,i)) y_a1(i)],[0 x_a3(i) ]  , [-R  y_a3(i)] );
        [xi_12, yi_12] = polyxpoly([R*cos(Q1(j,i)) x_a1(i)], [R*sin(Q1(j,i)) y_a1(i)],[R*cos(th2(i)) x_a2(i) ]  , [R*sin(th2(i))  y_a2(i)] ); 

        if abs((th1(i-1))-(Q1(j,i)))<0.1 && abs((R*cos(Q1(j,i))-x_a1(i))^2+(R*sin(Q1(j,i))-y_a1(i))^2 - L^2)<0.1 && isempty(xi_13) && isempty(xi_12)
            th1(i) = Q1(j,i);
            break
                   

        end
    end
end

q = [th1;th2;d3];