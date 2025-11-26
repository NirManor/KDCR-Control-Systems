function f2=f2_plan(prof,t)

global l2 d3 l4 l m2 m3 M 

f2=zeros(length(t),1);
q=q_plan(prof,t);
q_dot=q_dot_plan(prof,t,'diff');
q_dot2=q_dot2_plan(prof,t,'diff');

for i=1:length(t)

    th1 = q(i,1);
    th2 = q(i,2);
    d3 = q(i,3);
    th1d=q_dot(i,1);
    th2d=q_dot(i,2);
    d3d=q_dot(i,3);
    th1dd=q_dot2(i,1);

    f2(i)=(l2*m2*(th1dd*cos(th2) - 2*th1d*th2d*sin(th2)))/2 - M*((d3 + l4)*(th1dd*sin(th2) + th1d*th2d*cos(th2)) - l2*(th1dd*cos(th2) - 2*th1d*th2d*sin(th2)) + 2*d3d*th1d*sin(th2) + th1d*th2d*cos(th2)*(d3 + l4)) - m3*((th1dd*sin(th2) + th1d*th2d*cos(th2))*(d3 - l/2 + l4) - l2*(th1dd*cos(th2) - 2*th1d*th2d*sin(th2)) + 2*d3d*th1d*sin(th2) + th1d*th2d*cos(th2)*(d3 - l/2 + l4));

end