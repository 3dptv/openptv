function x2=rotateX(R,x)

si=size(x);
x2=zeros(si(1,1),3);

x2(1:si(1,1),1)=sum(repmat(R(1,:),si(1,1),1).*x,2);
x2(1:si(1,1),2)=sum(repmat(R(2,:),si(1,1),1).*x,2);
x2(1:si(1,1),3)=sum(repmat(R(3,:),si(1,1),1).*x,2);