function [x,y]=find_closest_points(x,y,xp,yp,tol)

for i=1:length(x)
   dx=sum((repmat([x(i) y(i)],length(xp),1)-[xp yp]).^2,2).^0.5; 
   mini=min(dx);
   if mini<tol
       ind=find(dx==mini);
       x(i)=xp(ind(1));
       y(i)=yp(ind(1));
   else
       x(i)=NaN;
       y(i)=NaN;
   end
end