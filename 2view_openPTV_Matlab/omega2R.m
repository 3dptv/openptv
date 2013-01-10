function R=omega2R(omega)
%get rotation matrix from rotation vector

p=sum(omega.^2)^0.5;
if p>0
    e=omega/p;
else
    e=[0 0 1];
end
R= [cos(p)+e(1)^2*(1-cos(p)) e(1)*e(2)*(1-cos(p))-e(3)*sin(p) e(1)*e(3)*(1-cos(p))+e(2)*sin(p);...
    e(2)*e(1)*(1-cos(p))+e(3)*sin(p) cos(p)+e(2)^2*(1-cos(p)) e(2)*e(3)*(1-cos(p))-e(1)*sin(p);...
    e(3)*e(1)*(1-cos(p))-e(2)*sin(p) e(3)*e(2)*(1-cos(p))+e(1)*sin(p) cos(p)+e(3)^2*(1-cos(p)) ];