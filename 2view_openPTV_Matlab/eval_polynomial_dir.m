function quality=eval_polynomial_dir(cam_pos)

global pos

global pix
global cam
global cam_dir
global omega
global poly_cal
global poly_x
global poly_y
global proj_x
global proj_y

global cam_const_v2
global im2world_const_v2
global poly_order

global lz
global target_xim 
global target_yim

pos(cam,1:3)=cam_pos;

% first, make dir with reference to camera
vec=-cam_pos;
base_vec=[0 0 -1];
tmp=cross(base_vec,vec/norm(vec));
mag=asin(norm(tmp));
if mag==0
    omega(cam,1:3)=[0 0 0];
else
    omega(cam,1:3)=mag*tmp/norm(tmp);
end

si=size(poly_cal);
dir=repmat(cam_pos,si(1,1),1)-poly_cal;
dir=dir./repmat(sum(dir.^2,2).^0.5,1,3);
R=omega2R(squeeze(-omega(cam,1:3)));
dir=rotateX(R,dir);

% and now use only x,y component of dir, z comp is redundant..

% linear
A=[repmat(1,si(1,1),1) dir(:,1:2)];
% add quadratic terms
if poly_order>1
    A=[A dir(:,1).^2 dir(:,2).^2 dir(:,1).*dir(:,2)];
end
% add cubic terms
if poly_order>2
    A=[A dir(:,1).^3 dir(:,2).^3 dir(:,1).^2.*dir(:,2) dir(:,2).^2.*dir(:,1)];
end

Y=[poly_x poly_y];

% A*C=Y
% A'*A*C=A'*Y
cam_const_v2{cam}=(A'*A)\(A'*Y);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
lz=0;
target_xim=pix(1)/2+0.5;
target_yim=pix(2)/2+0.5;
xy_0=[0 0];
LB      =[-0.05 -0.05];
UB      =[ 0.05  0.05];
ini_diff=eval_xy_for_z_xim_yim_poly(xy_0);
xy = fminsearchbnd(@eval_xy_for_z_xim_yim_poly, xy_0,LB,UB);
min_diff=eval_xy_for_z_xim_yim_poly(xy);
vec=[xy(1) xy(2) lz]-cam_pos;
cam_dir{cam}=vec/norm(vec);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% and now again, but with proper vec!!!!
base_vec=[0 0 -1];
tmp=cross(base_vec,vec/norm(vec));
mag=asin(norm(tmp));
if mag==0
    omega(cam,1:3)=[0 0 0];
else
    omega(cam,1:3)=mag*tmp/norm(tmp);
end

si=size(poly_cal);
dir=repmat(cam_pos,si(1,1),1)-poly_cal;
dir=dir./repmat(sum(dir.^2,2).^0.5,1,3);
R=omega2R(squeeze(-omega(cam,1:3)));
dir=rotateX(R,dir);

% and now use only x,y component of dir, z comp is redundant..

% linear
A=[repmat(1,si(1,1),1) dir(:,1:2)];
% add quadratic terms
if poly_order>1
    A=[A dir(:,1).^2 dir(:,2).^2 dir(:,1).*dir(:,2)];
end
% add cubic terms
if poly_order>2
    A=[A dir(:,1).^3 dir(:,2).^3 dir(:,1).^2.*dir(:,2) dir(:,2).^2.*dir(:,1)];
end

Y=[poly_x poly_y];

% A*C=Y
% A'*A*C=A'*Y
cam_const_v2{cam}=(A'*A)\(A'*Y);




[X2D]=world2im_poly_pos(poly_cal,cam_const_v2{cam},cam_pos,omega(cam,1:3));
xim=X2D(:,1);
yim=X2D(:,2);
de=sum(([poly_x poly_y]-[xim yim]).^2,2).^0.5;
quality=mean(de);