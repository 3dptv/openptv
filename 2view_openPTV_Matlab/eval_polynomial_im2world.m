function quality=eval_polynomial_im2world(cam_pos)

global pos
global cam
global cal
global common_ind
global im2world_x

global im2world_const_v2
global poly_order

pos(cam,1:3)=cam_pos(1:3);

si=size(cal(common_ind));
dir=cal(common_ind,1:3)-repmat(pos(cam,1:3),si(1,1),1);
dir=dir./repmat(sum(dir.^2,2).^0.5,1,3);

imx=im2world_x{cam}(common_ind,1);
imy=im2world_x{cam}(common_ind,2);

% and now fill matrices so that dir(1..3) is function of imx,imy

% linear
A=[repmat(1,si(1,1),1) imx imy];
% add quadratic terms
if poly_order>1
    A=[A imx.^2 imy.^2 imx.*imy];
end
% add cubic terms
if poly_order>2
    A=[A imx.^3 imy.^3 imx.^2.*imy imy.^2.*imx];
end

Y=[dir(:,1) dir(:,2) dir(:,3)];

% A*C=Y
% A'*A*C=A'*Y
im2world_const_v2{cam}=(A'*A)\(A'*Y);

%%%%%%%%%%%%%%%%%% im2wolrd part %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
scale=max(Y(:,2))-min(Y(:,2));
proj=A*im2world_const_v2{cam};
delta=sum((proj-Y).^2,2).^0.5/scale;
quality=mean(delta);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

