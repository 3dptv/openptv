function diff=eval_xy_for_z_xim_yim_poly(x)

global lz
global cam_const_v2
global cam
global pos
global omega
global target_xim 
global target_yim

X2d=world2im_poly_pos([x lz], cam_const_v2{cam}, pos(cam,1:3), omega(cam,1:3));
xim=X2d(:,1);
yim=X2d(:,2);
diff=((target_xim-xim)^2+(target_yim-yim)^2)^0.5;
