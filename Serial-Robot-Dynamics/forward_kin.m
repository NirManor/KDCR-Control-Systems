function x=forward_kin(q)

global l1 l2 l4

x=zeros(3,length(q(:,1)));

th1=q(:,1);
th2=q(:,2);
d3=q(:,3);

for i=1:length(q(:,1))
    c1=cos(th1(i));
    s1=sin(th1(i));
    c2=cos(th2(i));
    s2=sin(th2(i));
    d=d3(i);
    A01=[c1 -s1 0 0; 
        s1 c1 0 0; 
        0 0 1 l1; 
        0 0 0 1];
    A12=[c2 0 -s2 0; 
        0 1 0 0; 
        s2 0 c2 0; 
        0 0 0 1];
    A23=[1 0 0 l2; 
        0 1 0 0; 
        0 0 1 d; 
        0 0 0 1];
    A3t=[1 0 0 0; 
        0 1 0 0; 
        0 0 1 l4; 
        0 0 0 1];

    A02 = A01*A12;
    A03 = A02*A23;
    A0t = A03*A3t;

    x(:,i)=A0t(1:3,4);
end