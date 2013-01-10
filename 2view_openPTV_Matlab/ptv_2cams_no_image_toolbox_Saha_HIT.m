% % % photrack AG, June-Oktober 2012, Beat Lüthi
% % %
% % % this script is the main for job:2 view PTV system
% % % goal: like IfU tracking, but smart z-depth, i.e. tracking &
% % % correspondence is united, reducing ambiguities and thus allowing to have
% % % only 2 camera views
% % %

% % % % % % TO DO:



% 0) define / calibrate cam pos & omega

global cal
global corners
global pix
global x
global y

global poly_cal
global poly_x
global poly_y

global poly_order

global ind_V
global cam
global in_dir
global xim
global yim

global pos
global omega
global cam_const_v2
global im2world_const_v2
global im2world_x
global common_ind

global A

global points
global num_frames
global max_points_per_frame
global Hz

global track_tol
global pixel_tol
global accuracy_2d
global accuracy_3d
global tol
global fps
global min_traj_length

global dir_horizontal
global center_point
global dir_vertical

global num_glues
global jump_length
global point_proj
global fit_range

global holes
global final_traj
global domain_radius
global hole_radius
global baffle_y_pos baffle_id

global r0 radius_constraint x_constraint y_constraint

xim =cell(3);
yim = cell(3);
ind_V = cell(3);
ind_reach = cell(3);
is_reached = cell(3);
in_dir = cell(3);
cam_const_v2 =cell(3);

%%%%%%%%%% CONSTANTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%camera stuff:
pix=[512 512];                                  %(1,1)=horizontal=x, (1,2)=vertical(bottom->top)= y
dp=20e-6;                                        %[m]

%recording specific stuff
fps=250;
num_frames=200;
max_points_per_frame=2500;
path='C:\Users\Luethi\Documents\Dropbox\PTV_working_folders\Flow field-HIT\';

exp=['HIT_WF1'];
img_folder=['image/'];
res_folder=['res_HIT_13/'];
cal_folder=['cal_13/'];
cal_name{1}=[path,cal_folder,'Cam1'];
cal_name{2}=[path,cal_folder,'Cam3'];
num_view1=1;
num_view2=3;

%calibration
poly_order=3; 

%domain and flow params
domain_radius=0.015; %[m]
hole_radius=0.005; %[m]
Hz=1;%3.48;
domain_width=2*domain_radius;
max_vel=0.2;                                     %[m/s] <-----TUNING
tol_vz=0.05;                                      %[m/s] <-----TUNING
baffle_y_pos=[ -0.0820   -0.0541   -0.0262    0.0017    0.0296    0.0575];

%correspondence and tracking stuff
track_tol=3*(max_vel/fps);                       %safety * max_vel/fps [m]
pixel_tol=3*(max_vel/fps)/(domain_width/pix(1)); %safety * max_vel/fps [m] / domain_width/pix(1) [m/pix] --> [pix]
accuracy_2d=3;                                   %[pixel] <----------TUNING
accuracy_3d=5e-3;                                %[m] <--------------TUNING
tol=3*domain_width/pix(1)                        %[m] <--------------TUNING
min_traj_length=20;                              %<------------------TUNING
fit_range=10;                                    %<------------------TUNING   
kernel_width=20;                                 %<------------------TUNING
golay_order=2;

%for dispersion analysis
radius_constraint=1;
x_constraint=0;
y_constraint=0;
max_search_hopping=-20;
%%%%%%%%%% END OF CONSTANTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%% BEGIN OF CALIBRATION
%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
cam_const_name=[path,cal_folder,'cam_const_im2world.mat'];
if exist(cam_const_name)%1>2%
    disp('loading camera constants')
    load(cam_const_name)
else
    na=[path,cal_folder,'calblock.txt'];
    cal=load(na);
    ind=cal(:,1);
    cal=cal(:,2:4)*1e-3;
    corners=[find(ind==1) find(ind==21) find(ind==43) find(ind==63) find(ind==85) find(ind==105) ];
    
    figure;hold on;
    scatter3(cal(:,1),cal(:,2),cal(:,3));hold on;axis equal;
    scatter3(cal(corners(1),1),cal(corners(1),2),cal(corners(1),3),'r','filled');% shows point 1
    scatter3(cal(corners(2),1),cal(corners(2),2),cal(corners(2),3),'g','filled');% shows point 7
    scatter3(cal(corners(3),1),cal(corners(3),2),cal(corners(3),3),'b','filled');% shows point 36
    scatter3(cal(corners(4),1),cal(corners(4),2),cal(corners(4),3),'c','filled');% shows point 41
    scatter3(cal(corners(5),1),cal(corners(5),2),cal(corners(5),3),'m','filled');% shows point 141
    scatter3(cal(corners(6),1),cal(corners(6),2),cal(corners(6),3),'y','filled');% shows point 147
    title('those points need to be clicked in order r,g,b,c,m,y')
    axis([-0.01 0.03 -0.01 0.03 -0.01 0.03])
    view(16,66)
    
    for cam=1:2
        
        
        x=[];
        y=[];
        xp=[];
        yp=[];
        click_name=[path,cal_folder,'clicks_cam',num2str(cam),'.mat'];
        if exist(click_name)
            load(click_name)
            ca_na=[cal_name{cam},'.tif'];
            A{cam} = imread(ca_na);
        else
            ca_na=[cal_name{cam},'.tif'];
            A{cam} = imread(ca_na);
            
            
            if 1>2
                figure;imshow(A{cam});hold on;
                %%%%%this part can be replaced by _target files from 3D-PTV code
                se = strel('disk',1,0);
                filt1 = imopen(A{cam}, se);%
                %     figure;imshow(filt1);

                filt2=imextendedmax(filt1,30,4);%highe value of 2nd arg reduces num of detected points
                %     figure;imshow(filt2);

                CC = bwconncomp(filt2,4);
                num_Obj=CC.NumObjects
                S = regionprops(CC,'Centroid');
                pixel_val_xy = [S.Centroid]';
                tmp_xp = pixel_val_xy(1:2:end);
                tmp_yp = pixel_val_xy(2:2:end);
                %%%%%end of this part can be replaced by _target files from 3D-PTV code
            else
                %%%%%begin of replacement
                figure;image(A{cam});hold on;
                axis equal
                colormap(gray)
                
                targ_name=[cal_name{cam},'.tif_targets'];

                fid_t1 = fopen(targ_name);
                num_t1 = fscanf(fid_t1, '%i', [1 1]);
                f1=fscanf(fid_t1, '%i %f %f %i %i %i %i %i', [8 inf]);
                f1=f1';
                fclose(fid_t1);
                si_t1=size(f1);

                tmp_xp=f1(:,2);
                tmp_yp=f1(:,3);
                %%%%%end of replacement
            end
            
            scatter(tmp_xp,tmp_yp,'b')
            
            tx=['click the six points (r,g,b,c,m,y)'];
            disp(tx)
            [tmp_x,tmp_y] = ginput(6);
            [tmp_x,tmp_y]=find_closest_points(tmp_x,tmp_y,tmp_xp,tmp_yp,4);
            plot_ind=find(isnan(tmp_x)==0);
            scatter(tmp_x(plot_ind),tmp_y(plot_ind),'r','filled')
            %lscatter(tmp_x(plot_ind)+10,tmp_y(plot_ind)+10,ind(corners(plot_ind)),'TextColor','r')
            x=[x;tmp_x];
            y=[y;tmp_y];
            %I uses struct, since length of tmp_xp varies for each level
            xp=tmp_xp;
            yp=tmp_yp;
            save(click_name, 'x', 'y', 'xp', 'yp');
        end
        
        %now change to proper coordsystem x=horizontal, y=vertical(bottom->top)
        y=pix(2)-y+1;
        yp=pix(2)-yp+1;
        
        %---------polynomial part plus optimized pos and omega
        good_corners=find(isnan(x)==0);
        poly_cal=cal(corners(good_corners),1:3);
        poly_x=x(good_corners);
        poly_y=y(good_corners);

        poly_order=1;
        
        pos_0=[0. 0. 1];
        LB        =[-1 -1 0.1];
        UB        =[ 1  1 2];
        initial_quality_poly_pos = eval_polynomial_dir(pos_0)
        pos(cam,1:3)             = fminsearchbnd(@eval_polynomial_dir, pos_0,LB,UB);
        optimal_quality_poly_pos = eval_polynomial_dir(pos(cam,1:3)) % also this step is crucial, as it writes the cam_para_v2
        
        %-----end of polynomial part plus optimized pos and omega
        
        X2D = world2im_poly_pos(cal(corners(good_corners),1:3),cam_const_v2{cam},pos(cam,1:3),omega(cam,1:3));
        ximp = X2D(:,1);
        yimp = X2D(:,2);
        X2D = world2im_poly_pos(cal,                           cam_const_v2{cam},pos(cam,1:3),omega(cam,1:3));
        ximp_all = X2D(:,1);
        yimp_all = X2D(:,2);
        figure(3);imshow(A{cam});hold on;
        scatter(poly_x,pix(2)-poly_y+1,'b');
        scatter(ximp_all,pix(2)-yimp_all+1,'y');
        tx=['cal',num2str(cam),' from poly calibration, clicked points'];
        title(tx)
        axis equal
        
        [x,y]=find_closest_points(ximp_all,yimp_all,xp,yp,20);
        
        % now, use all points, since we are prette sure to have proper correspondence
        %---------again polynomial part plus optimized pos and omega BUT FOR ALL POINTS
        good_points=find(isnan(x)==0);
        poly_cal=cal(good_points,1:3);
        poly_x=x(good_points);
        poly_y=y(good_points);
        poly_order=3;
        
        pos_0=pos(cam,1:3);
        LB        =[-1 -1 0.1];
        UB        =[ 1  1 2];
        initial_quality_poly_pos = eval_polynomial_dir(pos_0)
        pos(cam,1:3)             = fminsearchbnd(@eval_polynomial_dir, pos_0,LB,UB);
        
        X2D = world2im_poly_pos(cal,cam_const_v2{cam},pos(cam,1:3),omega(cam,1:3));
        ximp_all = X2D(:,1);
        yimp_all = X2D(:,2);
        proj_x=ximp_all(good_points);
        proj_y=yimp_all(good_points);
        
        optimal_quality_poly_pos = eval_polynomial_dir(pos(cam,1:3)) % also this step is crucial, as it writes the cam_para_v2
        
        
        X2D = world2im_poly_pos(cal,cam_const_v2{cam},pos(cam,1:3),omega(cam,1:3));
        ximp_all = X2D(:,1);
        yimp_all = X2D(:,2);
        figure(3+cam);imshow(A{cam});hold on;
        scatter(poly_x,pix(2)-poly_y+1,'b');
        scatter(ximp_all,pix(2)-yimp_all+1,'y');
        
        [x,y]=find_closest_points(ximp_all,yimp_all,xp,yp,5);
        im2world_x{cam}(:,1)=x;
        im2world_x{cam}(:,2)=y;
        scatter(x,pix(2)-y+1,'m');
        
        yellow_points(cam,1:length(ximp_all),1)=ximp_all;
        yellow_points(cam,1:length(ximp_all),2)=yimp_all;
        
        tx=['cal',num2str(cam),' from poly calibration, all points'];
        title(tx)
        axis equal
        
    end
    
    %this ind has the same order as the calblock points so very useful!!!
    common_ind=find(isnan(im2world_x{1}(:,1))==0 & isnan(im2world_x{2}(:,1))==0);
        
    for cam=1:2
        pos_0=[0. 0. 1];%[pos(cam,1:3)];
        LB        =[-1 -1 0.1];
        UB        =[ 1  1 2  ];
        initial_quality_poly_pos = eval_polynomial_im2world(pos_0)
        pos(cam,1:3)             = fminsearchbnd(@eval_polynomial_im2world, pos_0,LB,UB);
        optimal_quality_poly_pos = eval_polynomial_im2world(pos(cam,1:3))
    end
    pos=pos
    
    dir1=im2world_poly_direct(im2world_x{1}(common_ind,1),im2world_x{1}(common_ind,2),1);
    dir2=im2world_poly_direct(im2world_x{2}(common_ind,1),im2world_x{2}(common_ind,2),2);
    [cal_im2world,dist]=intersectRays(pos(1,1:3),pos(2,1:3),dir1,dir2);
    
    av_ray_dist=mean(dist);
    delta=sum((cal_im2world(:,1:3)-cal(common_ind,1:3)).^2,2).^0.5;
    av_delta=mean(delta);
    tx=['av ray mismatch: ',num2str(av_ray_dist*1e6),'microns, \Delta: ',num2str(av_delta*1e3),'mm'];
    figure; hold on;
    scatter3(cal_im2world(:,1),cal_im2world(:,2),cal_im2world(:,3),'b')
    scatter3(cal(common_ind,1),cal(common_ind,2),cal(common_ind,3),5,'r','filled')
    axis equal;
    box on;
    title(tx)
    
    save(cam_const_name, 'im2world_const_v2','pos','yellow_points');%%,'f','dp'
    reply = input('ok, close all figures? Y/N [Y]: ', 's');
    if isempty(reply)
        reply = 'Y';
    end
    if reply=='y' | reply=='Y'
        close all;
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%% END OF CALIBRATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%  prepare for making co-planar-epipolar-values of both camviews
dir_a=im2world_poly_direct(pix(1)/2+0.5,pix(2)/2+0.5,1);
dir_b=im2world_poly_direct(pix(1)/2+0.5,pix(2)/2+0.5,2);
dir_vertical=cross(dir_a,dir_b);
dir_vertical=dir_vertical/norm(dir_vertical);
dir_horizontal=pos(2,1:3)-pos(1,1:3);
dir_horizontal=dir_horizontal/norm(dir_horizontal);
[center_point,dist]=intersectRays(pos(1,1:3),pos(2,1:3),dir_a,dir_b);
        

%%%%%%%%%%%%%%%%%NOW WE CAN START WORKING!!!!!%%%%%%%%%%%%%%%%%%%%%%%%%%      
%%%%%%%%%%%%%%%%%NOW WE CAN START WORKING!!!!!%%%%%%%%%%%%%%%%%%%%%%%%%%  
%%%%%%%%%%%%%%%%%NOW WE CAN START WORKING!!!!!%%%%%%%%%%%%%%%%%%%%%%%%%%

points_name=[path,res_folder,'read_points.mat'];
if exist(points_name)%
    disp('loading determined 2d points')
    load(points_name)
    tmp=size(points);
    max_points_per_frame=tmp(1,3);
    num_frames=tmp(1,2);
else
    % % % read in points!!!!
    points=zeros(2,num_frames,max_points_per_frame,5);%cams,num frames,num points,[x,y,vertical_val,occ]
    for frame=1:num_frames
        
        tx=['preparing frame: ',num2str(frame)];
        disp(tx);
        
%         targ_name{1}=[path,img_folder,exp,'_cam',num2str(1),'.',num2str(frame),'_targets'];
%         targ_name{2}=[path,img_folder,exp,'_cam',num2str(2),'.',num2str(frame),'_targets'];
        clear targ_name
        targ_name{1}=[path,img_folder,'Cam',num2str(num_view1),'.',num2str(frame),'_targets'];
        targ_name{2}=[path,img_folder,'Cam',num2str(num_view2),'.',num2str(frame),'_targets'];
        
        fid_t1 = fopen(targ_name{1});
        fid_t2 = fopen(targ_name{2});
        num_t1 = fscanf(fid_t1, '%i', [1 1]);
        num_t2 = fscanf(fid_t2, '%i', [1 1]);
        f1=fscanf(fid_t1, '%i %f %f %i %i %i %i %i', [8 inf]);
        f1=f1';
        fclose(fid_t1);
        si_t1=size(f1);
        f2=fscanf(fid_t2, '%i %f %f %i %i %i %i %i', [8 inf]);
        f2=f2';
        fclose(fid_t2);
        si_t2=size(f2);
        
        points(1,frame,1:num_t1,1:2)=f1(:,2:3);
        points(1,frame,1:num_t1,2)=pix(2)-points(1,frame,1:num_t1,2)+1;%%<----PTV measures y from top also....
        points(1,frame,max_points_per_frame,4)=num_t1;
        
        % already prepare vertical_val, doing it only once and for all.        
        dir=im2world_poly_direct(...
            squeeze(points(1,frame,1:points(1,frame,max_points_per_frame,4),1)),...
            squeeze(points(1,frame,1:points(1,frame,max_points_per_frame,4),2)),1);
        [point,t]=intersectionPlaneLine_vec(pos(1,1:3),dir_horizontal,dir,center_point,dir_vertical);
        points(1,frame,1:points(1,frame,max_points_per_frame,4),3)=t;
        
        points(2,frame,1:num_t2,1:2)=f2(:,2:3);
        points(2,frame,1:num_t2,2)=pix(2)-points(2,frame,1:num_t2,2)+1;%%<----PTV measures y from top also....
        points(2,frame,max_points_per_frame,4)=num_t2;
        
        % already prepare vertical_val, doing it only once and for all.        
        dir=im2world_poly_direct(...
            squeeze(points(2,frame,1:points(2,frame,max_points_per_frame,4),1)),...
            squeeze(points(2,frame,1:points(2,frame,max_points_per_frame,4),2)),2);
        [point,t]=intersectionPlaneLine_vec(pos(1,1:3),dir_horizontal,dir,center_point,dir_vertical);
        points(2,frame,1:points(2,frame,max_points_per_frame,4),3)=t;
    end
    save(points_name, 'points');
end

tot_num_points=sum(points(1,1:num_frames,max_points_per_frame,4));

name_traj=[path,res_folder,'trajectories_',exp,'.mat'];
if exist(name_traj)
    disp('loading computed trajectories')
    load(name_traj);
%     compute_holes();
%     disp('rendering trajectories')
%     render_traj_figures(0);
else
    %%%now find matches etc,
    final_traj=[];
    traj_count=0;
    num_glues=0;
    jump_length=zeros(500000,1);
    dist_to_end=zeros(500000,1);
    dist_to_beg=zeros(500000,1);
    for frame=1:num_frames-1
        tx=['computing traj that start from frame ',num2str(frame)];
        disp('================================================================')
        disp(tx);
        count=0;
        tic
        for pid1=1:points(1,frame,max_points_per_frame,4)
            if points(1,frame,pid1,4)==0 %% not already occupied
                %we have a point to deal with
                
                vertical_1_2=points(1,frame,pid1,3);
                vertical_2_1=squeeze(points(2,frame,1:points(2,frame,max_points_per_frame,4),3));
                cand_list=find(abs(vertical_1_2-vertical_2_1)<tol & squeeze(points(2,frame,1:points(2,frame,max_points_per_frame,4),4))==0);
                
                le=length(cand_list);
                if le>0
                    %%now we start tracking etc
                    traj=cell(le,1);
                    traj_length=[];
                    for i=1:le
                        [points3D,dist]=get_3d_point_direct(frame,pid1,cand_list(i));
                        if dist<tol
                            if points3D(1,3)>-0.005 & points3D(1,3)<0.02%(points3D(1,1)^2+points3D(1,3)^2)^0.5<domain_radius % is it within domain?
                                pid2=cand_list(i);
                                [traj{i},reason{i}]=get_track_glue_v2(frame,pid1,pid2,points3D(1,:),dist);%
                                len=size(traj{i});
                                traj_length(i)=len(1,1);
                            else
                                traj_length(i)=0;
                            end
                        else
                            traj_length(i)=0;
                            reason{1}=[0 0 0 0 0 dist*1e3 0];
                        end
                    end
                    %now sort out what we have
                    [traj_length,k]=sort(traj_length,'descend');
                    
                    %%%%why did it stop?????????????
                    rea=reason{k(1)};
                    tx=['track length: ',num2str(rea(1)),', '];
                    if rea(2)==1 & rea(3)==0
                        tx=[tx,'lost track cam1, '];
                    end
                    if rea(4)==1 & rea(5)==0
                        tx=[tx,'lost track cam2, '];
                    end
                    if rea(6)>0
                        tx=[tx,'ray mismatch: ',num2str(rea(6)),'mm, '];
                    end
                    if rea(7)>0
                        tx=[tx,'track mismatch: ',num2str(rea(7)),'mm'];
                    end
                    %disp(tx)
                    aa=1;
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    
                    if traj_length(1)>min_traj_length
                        if le>1
                            if traj_length(1)>2*traj_length(2)
                                best_ind=1;
                            else
                                ind=find(traj_length>min_traj_length);
                                best_dist=1000;
                                best_ind=-1;
                                for ii=ind
                                    quality=mean(traj{k(ii)}(:,7));
                                    if quality<best_dist
                                        best_dist=quality;
                                        best_ind=ii;
                                    end
                                end
                            end
                        else
                            best_ind=1;
                        end
                    else
                        best_ind=-1;
                    end
                    if best_ind>-1
                        
                        
                        px = smooth(traj{k(best_ind)}(:,4),kernel_width,'sgolay',golay_order);
                        py = smooth(traj{k(best_ind)}(:,5),kernel_width,'sgolay',golay_order);
                        pz = smooth(traj{k(best_ind)}(:,6),kernel_width,'sgolay',golay_order);
                        vx  = (px(2:end)-px(1:end-1))*fps;
                        vy  = (py(2:end)-py(1:end-1))*fps;
                        vz  = (pz(2:end)-pz(1:end-1))*fps;
                        vx=[vx(1);vx];
                        vy=[vy(1);vy];
                        vz=[vz(1);vz];
                        if max(abs(vz))<tol_vz
                            traj_count=traj_count+1;
                            traj_length_stat(traj_count)=traj_length(best_ind);
                            dist_travelled(traj_count)=sum(1/fps*(vx.^2+vy.^2+vz.^2).^0.5);
                            if sum(traj{k(best_ind)}(:,9))>0
                                num_glues=num_glues+1;
                                jump_length(num_glues)=sum(traj{k(best_ind)}(:,9));
                                ind=find(traj{k(best_ind)}(:,9)>0);
                                dist_to_beg(num_glues)=ind(1)-1;
                                dist_to_end(num_glues)=traj_length(best_ind)-ind(end);
                            end
                            
                            final_traj=[final_traj;traj{k(best_ind)}(:,[1 2 3 7 8]) px py pz vx vy vz traj{k(best_ind)}(:,9)];%lost columnfor glued or not
                            count=count+1;
                            disp(tx)
                            tx_0 =['-------------------------------------------'];
                            tx_1 =['frame:...................',num2str(frame)];
                            tx_2 =['# points:................',num2str(points(1,frame,max_points_per_frame,4))];
                            tx_3 =['pid1:....................',num2str(pid1)];
                            tx_4 =['# traj from frame:.......',num2str(count)];
                            tx_4b=['cum. 2d->linked 3d (%):..',num2str(round(100*length(final_traj)/tot_num_points))];
                            tx_5 =['# traj total:............',num2str(traj_count)];
                            tx_6 =['mean traj length:........',num2str(mean(traj_length_stat(1:traj_count)))];
                            tx_7 =['# traj with glue:........',num2str(num_glues)];
                            tx_8 =['mean glues:..............',num2str(mean(jump_length(1:num_glues)))];
                            tx_9 =['dist to beg 1st glue:....',num2str(mean(dist_to_beg(1:num_glues)))];
                            tx_10=['dist to end last glue:...',num2str(mean(dist_to_end(1:num_glues)))];
                            disp(tx_0)
                            disp(tx_1)
                            disp(tx_2)
                            disp(tx_3)
                            disp(tx_4)
                            disp(tx_0)
                            disp(tx_4b)
                            disp(tx_5)
                            disp(tx_6)
                            disp(tx_7)
                            disp(tx_8)
                            disp(tx_9)
                            disp(tx_10)
                            disp(tx_0)
                            
                            if traj_length(best_ind)>100 & pid1>10000000
                                %some times also because PTV sees one, where
                                %there should be two particles
                                %%% add reason of loss to title!
                                if (rea(6)>0 | rea(7)>0) & ~isempty(point_proj)
                                    render_proj=1;
                                else
                                    render_proj=0;
                                end
                                render_situation(traj{k(best_ind)},1,path,'img_1Hz_rec3/streaks_2000fps_1Hz_rec3_cam1.',render_proj);
                                aaa=1;
                            end
                            
                            % update occupied stuff
                            for i=1:traj_length(best_ind)
                                fra=traj{k(best_ind)}(i,1);
                                pi1=traj{k(best_ind)}(i,2);
                                pi2=traj{k(best_ind)}(i,3);
                                points(1,fra,pi1,4)=1;
                                points(2,fra,pi2,4)=1;
                            end
                        else
                            disp('dropped out, too high vz')
                        end
                    end
                end
            else
                aa=1;
            end
        end
        time_for_frame=toc;
        
        if 1>2%frame==round(num_frames/3) | frame==round(2*num_frames/3)
            
            compute_holes();
            render_traj_figures(0,246,30,0.08);
            
            figure(46);
            [nout,xout]=nhist(traj_length_stat(1:traj_count),100);
            loglog(xout,nout);
            xlabel('traj length (frames)');
            ylabel('pdf');
            tx=['num traj: ',num2str(traj_count)];
            title(tx);
            
            figure(47);
            [nout,xout]=nhist(dist_travelled(1:traj_count),100);
            loglog(xout,nout.*xout);
            xlabel('mean dist travelled (m)');
            ylabel('pdf\cdotx');
            tx=['num traj: ',num2str(traj_count)];
            title(tx);
            
        end
        aa=1;
    end
    save(name_traj,'final_traj');
end


load(name_traj);
% compute_holes();
render_traj(150);
% render_traj_figures(1,246,30,0.08);
 

% %now we should generously glue the trajectories
% name_glue_traj=[path,res_folder,'glued_trajectories_',exp,'.mat'];
% glue_all_traj_3d();
% re_calc_vel();
% save(name_glue_traj,'final_traj');


% %now we can compute dispersion
% name_glue_traj=[path,res_folder,'glued_trajectories_',exp,'.mat'];
% load(name_glue_traj);
% % 
% r0=6e-3;
% if radius_constraint>domain_radius
%     name_dispersion=[path,res_folder,'pair_dispersion_',num2str(r0),'_no_constraint_',exp,'.mat']; 
% else
%     name_dispersion=[path,res_folder,'pair_dispersion_',num2str(r0),'_',exp,'.mat'];
% end
% disp('computing pair dispersion')
% calc_dispersion(name_dispersion);
% r0=8e-3;
% if radius_constraint>domain_radius
%     name_dispersion=[path,res_folder,'pair_dispersion_',num2str(r0),'_no_constraint_',exp,'.mat']; 
% else
%     name_dispersion=[path,res_folder,'pair_dispersion_',num2str(r0),'_',exp,'.mat'];
% end
% disp('computing pair dispersion')
% calc_dispersion(name_dispersion);
% r0=10e-3;
% if radius_constraint>domain_radius
%     name_dispersion=[path,res_folder,'pair_dispersion_',num2str(r0),'_no_constraint_',exp,'.mat']; 
% else
%     name_dispersion=[path,res_folder,'pair_dispersion_',num2str(r0),'_',exp,'.mat'];
% end
% disp('computing pair dispersion')
% calc_dispersion(name_dispersion);
% 
% if radius_constraint>domain_radius
%     name_dispersion=[path,res_folder,'sinlge_dispersion_no_constraint_',exp,'.mat']; 
% else
%     name_dispersion=[path,res_folder,'single_dispersion_',exp,'.mat'];
% end
% disp('computing pair dispersion')
% calc_single_dispersion(name_dispersion);
% 
% name_baffle=[path,res_folder,'baffle_id_',exp,'.mat'];
% if exist(name_baffle)
%     load(name_baffle);
% else
%     calc_baffle_id(name_baffle);
%     load(name_baffle);
% end
% if radius_constraint>domain_radius
%     name_dispersion=[path,res_folder,'sinlge_dispersion_hopping_no_constraint_',exp,'.mat']; 
% else
%     name_dispersion=[path,res_folder,'single_dispersion_hopping_',exp,'.mat'];
% end
% disp('computing pair dispersion')
% calc_single_dispersion_hopping(name_dispersion,max_search_hopping,10e-3);

% load(name_traj);
% compute_holes();
% name_field=[path,res_folder,'base_cell_',exp,'.mat'];
% disp('rendering base cell')
% render_base_cell(name_field,2);



% %check howmany traj go through baffle holes, if any?
% area=0.05^2*pi;
% hole_area=0.005^2*pi;
% all_holes_area=12*hole_area;
% area_fraction=all_holes_area/area;
% compute_holes();
% for baff_id=1:6
%     through_holes{baff_id}=find_through_baffle(baff_id);
%     si=size(through_holes{baff_id});
%     if si(1,1)>0
%         figure(100+baff_id);hold on;
%         scatter3(through_holes{baff_id}(:,1),through_holes{baff_id}(:,2),through_holes{baff_id}(:,3),'m','filled');
%     end
%     axis equal
%     [baff_id si(1,1) area_fraction*si(1,1)]
% end


% %make movie
% trail=300;
% count=0;
% clear mov_frames;
% for i=trail:2:num_frames
%     ind=find(final_traj(:,1)>i-trail & final_traj(:,1)<=i);
%     le=length(ind);
%     [i le]
%     velo=(final_traj(ind,9).^2+final_traj(ind,10).^2+final_traj(ind,11).^2).^0.5;
%     figure(100);
%     scatter3(final_traj(ind,6),-final_traj(ind,8),final_traj(ind,7),5,velo,'filled');
%     xlabel('x');
%     ylabel('-z');
%     zlabel('y');
%     colorbar;
%     box on;
%     axis equal;
%     caxis([0 0.6])
%     axis([-0.05 0.05 -0.05 0.05 -0.1 0.1])
%     view(-157,20)
%     count=count+1;
%     mov_frames(count) = getframe(gcf);
% end
% mov_name=[path,'res_1Hz_rec2/traj_movie_1Hz_rec2.avi'];
% movie2avi(mov_frames(1:count), mov_name)

   
    
    

