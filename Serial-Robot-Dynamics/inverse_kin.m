function q=inverse_kin(x,elbows)

global l1 l2 l4

Px = x(1,:)';
Py = x(2,:)';
Pz = x(3,:)';

th1 = atan2(Py.*elbows(1),Px.*elbows(1));

A=Py+sin(th1)*l2;
B=2*(l1*sin(th1)-Pz.*sin(th1));
C=sin(th1)*l2-Py;
delta=B.^2-4*A.*C;
delta_s=delta.^0.5;
th2_tmp=[2*atan((-B+delta_s)./(2*A)) 2*atan((-B-delta_s)./(2*A))];
th2=0.5*(-min(th2_tmp,[],2).*(elbows(:,2)-1)+max(th2_tmp,[],2).*(elbows(:,2)+1));

d3=(sin(th1).*cos(th2)*l2-Py)./(sin(th1).*sin(th2))-l4;

q=[th1 th2 d3];

end