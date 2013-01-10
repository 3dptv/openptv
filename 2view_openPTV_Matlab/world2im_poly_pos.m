function X2d=world2im_poly_pos(w_points,cam_const_v2,cam_pos,cam_omega)

global poly_order

% first, make dir with reference to camera

si=size(w_points);
dir=repmat(cam_pos,si(1,1),1)-w_points;
dir=dir./repmat(sum(dir.^2,2).^0.5,1,3);
world_dir=dir;
R=omega2R(squeeze(-cam_omega));
dir=rotateX(R,dir);

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


Y=A*cam_const_v2;
xim(:,1)=Y(:,1);
yim(:,1)=Y(:,2);
X2d=[xim yim -world_dir];

