function [x_odd,x_even]=calc_baffle_holes(figid,baffle_y_pos,domain_radius)

%define holes geometry
s=0.0254;%0.024;
offset=[0 0.005];
off_al=-2;


di=[s;0];
dj=[s/2;3^0.5/2*s];
origo=[-s/2;-3^0.5/2/3*s];
c=0;
for i=-2:1
    for j=-2:1
        x=i*di+j*dj-origo;
        if sum(x.^2)^0.5<1*0.9*domain_radius 
            c=c+1;
            x_odd(c,1:2)=x;
        end
    end
end
R=[cosd(60) sind(60);-sind(60) cosd(60)];
for i=1:c
    x_even(i,:)=(R*x_odd(i,:)')';
end

R=[cosd(off_al) sind(off_al);-sind(off_al) cosd(off_al)];
for i=1:c
    x_even(i,:)=(R*x_even(i,:)')';
    x_odd(i,:) =(R*x_odd(i,:)')';
end

x_even=x_even+repmat(offset,length(x_even),1);
x_odd=x_odd+repmat(offset,length(x_odd),1);


figure(figid);hold on;
scatter(0,0,10,'r','filled')
scatter(x_odd(:,1),x_odd(:,2),10,'g');
scatter(x_even(:,1),x_even(:,2),10,'b');
axis equal
xlim([-0.05 0.05])
ylim([-0.05 0.05])
legend('center','holes for odd baffles','holes for even baffles')

%end of define holes geometry
