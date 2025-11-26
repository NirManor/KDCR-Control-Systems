function X=forward_kin(q)
% This function calculates the forward kinematics.
% q is a vector with the actuators position: [teta1,teta2, d3].
% The function returns the Tool position with the format: [x,y,phi].
% X is matrix with i rows according to the number of solution branch.

% close all;clear all;clc


syms x y r phi L R b1_x b1_y b2_x b2_y b3_x b3_y theta1 theta2  d3 t;

S1 = sin(q(1));C1 = cos(q(1));S2 = sin(q(2));C2 = cos(q(2));D3=q(3);

r=2; R=4; L=3.5;

a1_x = x + r*cos(phi+pi/3);
a1_y = y + r*sin(phi+pi/3);
b1_x = R*C1;
b1_y = R*S1;

eq1 = expand((a1_x-b1_x)^2+(a1_y-b1_y)^2==L^2);

a2_x = x;
a2_y = y;
b2_x = R*C2;
b2_y = R*S2;

eq2 = (a2_x-b2_x)^2+(a2_y-b2_y)^2==L^2;

a3_x = x + r*cos(phi);
a3_y = y + r*sin(phi);
b3_x = 0;
b3_y = -R;

eq3= (a3_x-b3_x)^2+(a3_y-b3_y)^2==D3^2;

eq1_2=eq1-eq2;
eq2_3=eq2-eq3;

[X_temp,Y_temp]=solve(eq1_2,eq2_3,[x,y]);

ANS=((X_temp^2+Y_temp^2+R^2-2*X_temp*R*C2-2*Y_temp*R*S2-L^2));
ANS = subs(ANS,sin(phi),(2*t)/(1+t^2));
ANS = subs(ANS,cos(phi),(1-t^2)/(1+t^2));

ANS = collect((ANS),t);
[N,D] = numden(ANS);
NUM = coeffs(N,t);
NUM(1:end) = NUM(end:-1:1);
NUM = round(NUM/NUM(1),16);
NUM_round=round(NUM,4)

DOM = coeffs(D,t);
DOM(1:end) = DOM(end:-1:1);
DOM = round(DOM/DOM(1),16);
DOM_round=round(DOM,4)
r_a = real(roots(NUM))
i_a = imag(roots(NUM))
r_b = real(roots(DOM))
sol_test = vpasolve(N,t);
sol_temp=vpasolve(ANS,t)
sol_temp_round=round(sol_temp,5)

j = 0;
for i= 1:size(r_a)
    
    if (double(i_a(i)) == 0)
        j = j+1;
        r(j) = (r_a(i));
    end
end

r;

for i = 1:j
    cos_fi = (1-r(i)^2)/(1+r(i)^2);
    sin_fi = (2*r(i))/(1+r(i)^2);
    Fi(i) = atan2(sin_fi,cos_fi);
end

Fi;
deg=rad2deg(double(Fi))

for i =1:j
    X(i,1)=(double(subs(X_temp,phi,Fi(i))));
    X(i,2)=(double(subs(Y_temp,phi,Fi(i))));
    X(i,3)=(double(Fi(i)));
end    

deg=rad2deg(double(X));









% 
% 
% 
% r=2; R=4; L=3.5;
% S1 = sin(q(1));C1 = cos(q(1));S2 = sin(q(2));C2 = cos(q(2));D3=q(3);
% 
% A = 2*R*C1 - 2*R*C2 - r*cos(phi) + 3^(1/2)*r*sin(phi);
% B = 2*R*S2 - 2*R*S1 + r*sin(phi) + 3^(1/2)*r*cos(phi);
% E = R*r*cos(phi)*C1 - r^2 + R*r*sin(phi)*S1 + 3^(1/2)*R*r*cos(phi)*S1 - 3^(1/2)*R*r*C1*sin(phi);
% C = - 2*R*C2 - 2*r*cos(phi);
% D = - 2*R - 2*R*S2 - 2*r*sin(phi);
% F = L^2 - D3^2 + r^2 + 2*R*sin(phi)*r;
% 
% 
% % A = 2*R*c1 - 2*R*cos(c2) - r*cos(phi) + 3^(1/2)*r*sin(phi);
% % B = 2*R*s2 - 2*R*s1 + r*sin(phi) + 3^(1/2)*r*cos(phi);
% % E = R*r*cos(phi)*c1 - r^2 + R*r*sin(phi)*s1 + 3^(1/2)*R*r*cos(phi)*s1 - 3^(1/2)*R*r*c1*sin(phi);
% % C = - 2*R*c2 - 2*r*cos(phi);
% % D = - 2*R - 2*R*s2 - 2*r*sin(phi);
% % F = L^2 - d3^2 + r^2 + 2*R*sin(phi)*r;
% 
% 
% X = ((D*E-B*F)/(A*D-B*C));
% Y = ((A*F-C*E)/(A*D-B*C));
% 
% 
% K = ((X^2+Y^2+R^2-2*X*R*C2-2*Y*R*S2-L^2));
% K = subs(K,sin(phi),(2*t)/(1+t^2));
% K = (subs(K,cos(phi),(1-t^2)/(1+t^2)));
% 
% % ans= vpasolve(K, t)
% 
% % K = subs(K,sin(theta1),S1);
% % K = subs(K,cos(theta1),C1);
% % K = subs(K,sin(theta2),S2);
% % K = subs(K,cos(theta2),C2);
% % K = subs(K,d3,D3);
% 
% % K = subs(K,r,2);
% % K = subs(K,R,4);
% % K = (subs(K,L,3.5));
% 
% ANS = collect((K),t);
% 
% 
% [N,D] = numden(ANS);
% 
% NUM = coeffs(N,t);
% NUM(1:end) = NUM(end:-1:1);
% NUM = round(NUM/NUM(1),5)
% 
% DOM = coeffs(D,t);
% DOM(1:end) = DOM(end:-1:1);
% DOM = round(DOM/DOM(1),5)
% 
% r_a_1=round(roots(NUM),5)
% r_b_1= round(roots(DOM),5)
% 
% 
% r_a = round(real(roots(NUM)),5);
% i_a = round(imag(roots(NUM)),5);
% r_b = round(real(roots(DOM)),5);
% 
% 
% 
% 
% 
% % NUM = coeffs(N,t);
% % NUM(1:end) = NUM(end:-1:1);
% % NUM = round(NUM/NUM(1),16);
% % 
% % DOM = coeffs(D,t);
% % DOM(1:end) = DOM(end:-1:1);
% % DOM = round(DOM/DOM(1),16);
% 
% r_a = real(roots(NUM));
% i_a = imag(roots(NUM));
% r_b = real(roots(DOM));
% 
% j = 0;
% for i= 1:size(r_a)
%     
%     if (double(i_a(i)) == 0 && double(r_a(i)) ~= double(r_b(i)))
%         j = j+1;
%         r(j) = round(r_a(i),4);
%     end
% end
% 
% 
% 
% % the roots of the numerator (2 had imaginary element so we ignored them as they aren't phisical solutions)
% for i = 1:j
%     cos_fi = (1-r(i)^2)/(1+r(i)^2);
%     sin_fi = (2*r(i))/(1+r(i)^2);
%     Fi(i) = atan2(sin_fi,cos_fi);
% end
% 
% fi_deg = round(Fi*180/pi,4)
% 
% for i=1:j
%       
%     fi=Fi(i);r=2;R=4;L=3.5;
%     
%     A = 2*R*C1 - 2*R*C2 - r*cos(fi) + 3^(1/2)*r*sin(fi);
%     B = 2*R*S2 - 2*R*S1 + r*sin(fi) + 3^(1/2)*r*cos(fi);
%     E = R*r*cos(fi)*C1 - r^2 + R*r*sin(fi)*S1 + 3^(1/2)*R*r*cos(fi)*S1 - 3^(1/2)*R*r*C1*sin(fi);
%     C = - 2*R*C2 - 2*r*cos(fi);
%     D = - 2*R - 2*R*S2 - 2*r*sin(fi);
%     F = L^2 - D3^2 + r^2 + 2*R*sin(fi)*r;
% 
% 
% 
% %     A = -r*cos(fi)-sqrt(3)*r*sin(fi)+2*R*(C3-C1);
% %     B = -r*sin(fi)+sqrt(3)*r*cos(fi)+2*R*(S3-S1);
% %     E = (C1+sqrt(3)*S1-2*C3)*R*r*cos(fi)+(-sqrt(3)*C1+S1-2*S3)*R*r*sin(fi);
% %     C = 2*r*cos(fi)+2*R*(C2-C3);
% %     D = 2*r*sin(fi)+2*R*(S2-S3);
% %     F = -1*r^2+2*R*r*C3*cos(fi)+2*R*r*S3*sin(fi);
% 
%     X(i,1) = double(((D*E-B*F)/(A*D-B*C)));
%     X(i,2) = double(((A*F-C*E)/(A*D-B*C)));
%     X(i,3) = double(fi);
%      
% end

