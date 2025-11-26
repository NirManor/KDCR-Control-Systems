function [J_x, J_q] = Jacobian_calculation(x, y, r, phi, L, R, theta1, theta2, d3)

% This function calculate parametrically the Jacobian for the tool space
% J_x and for the joint space J_q

F_1 = (x+r*cos(phi+pi/3)-R*cos(theta1))^2+(y+r*sin(phi+pi/3)-R*sin(theta1))^2-L^2;
F_2 = (x-R*cos(theta2))^2+(y-R*sin(theta2))^2-L^2;
F_3 = (x+r*cos(phi))^2+(y+r*sin(phi)-R)^2-d3^2;
F = [F_1; F_2;F_3];

df_x = simplify(diff(F, x));
df_y = simplify(diff(F, y));
df_phi = simplify(diff(F, phi));
J_x = [df_x  df_y  df_phi];

df_theta1 = simplify(diff(F, theta1));
df_theta2 = simplify(diff(F, theta2));
df_d3 = simplify(diff(F, d3));
J_q = [df_theta1  df_theta2  df_d3];

end