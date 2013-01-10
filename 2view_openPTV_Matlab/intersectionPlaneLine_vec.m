function [point,t]=intersectionPlaneLine_vec(p0,p1,p2,l0,l1)

% % % p2 is the directection matrix

% % % plane is point p0 + c1*p1 + c2*p2
% % % line is point l0 + c3*l1

% % % make sure everybody is a vector, i.e. size=(3,1)
si=size(p0);
if si(1,1)==1
    p0=p0';
end
si=size(p1);
if si(1,1)==1
    p1=p1';
end
si=size(l0);
if si(1,1)==1
    l0=l0';
end
si=size(l1);
if si(1,1)==1
    l1=l1';
end




% % % now solve for c, p0+c1*p1+c2*p2==l0+c3*l1
% % % now solve for c, p0+c1*p1+c2*p2==l0+c3*l1
% % % now solve for c, p0+c1*p1+c2*p2==l0+c3*l1
% % % now solve for c, p0+c1*p1+c2*p2==l0+c3*l1
A=zeros(3*length(p2),3*length(p2));
for i=1:length(p2)
    A((i-1)*3+1:(i-1)*3+3,(i-1)*3+1:(i-1)*3+3)=[p1 p2(i,1:3)' -l1];
    Y((i-1)*3+1:(i-1)*3+3,1)=l0-p0;
end

% A=[p1 p2 -l1];
% Y=l0-p0;
c=A\Y;
c=reshape(c,3,length(p2))';
% % % and now construct the point, e.g. using the line parametrization
point=repmat(l0',length(p2),1)+c(:,3)*l1';
t=c(:,3);