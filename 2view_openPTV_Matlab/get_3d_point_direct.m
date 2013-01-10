function [points3D,dist]=get_3d_point_direct(frame,pid1,pid2)

global points
global pos


dir1=im2world_poly_direct(points(1,frame,pid1,1),points(1,frame,pid1,2),1);
dir2=im2world_poly_direct(points(2,frame,pid2,1),points(2,frame,pid2,2),2);
[points3D,dist]=intersectRays(pos(1,1:3),pos(2,1:3),dir1,dir2);