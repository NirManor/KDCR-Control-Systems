function J=jacobian_mat(q)

th1 = q(1);
th2 = q(2);
d3 = q(3);

J = zeros(6,3);
J(1,1) = d3*sin(th1)*sin(th2) - l2*cos(th2)*sin(th1) + l4*sin(th1)*sin(th2);
J(2,1) = l2*cos(th1)*cos(th2) - d3*cos(th1)*sin(th2) - l4*cos(th1)*sin(th2);
J(6,1) = 1;
J(2,1) = -cos(th1)*(d3*cos(th2) + l4*cos(th2) + l2*sin(th2));
J(2,2) = -sin(th1)*(d3*cos(th2) + l4*cos(th2) + l2*sin(th2));
J(2,3) = l2*cos(th2) - d3*sin(th2) - l4*sin(th2);
J(2,4) = sin(th1);
J(2,5) = -cos(th1);
J(3,1) = -cos(th1)*sin(th2);
J(3,2) = -sin(th1)*sin(th2);
J(3,3) = cos(th2);

end