function [traj,reason]=get_track_glue_v2(frame,pid1,pid2,start_point,start_dist)

global points
global pix
global pos
global vLUT
global num_frames
global max_points_per_frame
global track_tol
global pixel_tol
global domain_radius
global tol
global fps
global min_traj_length
global fit_range


traj=[frame pid1 pid2 start_point start_dist 0 0];

ok=1;

while ok==1
    
    ok1=0;
    ok2=0;
    glued1=0;
    glued2=0;
    point_proj=[];

    frame=frame+1;
    
    si=size(traj);
    le=si(1,1);
    if le<=fit_range
        
        %find close by points to pid1 since traj still too short
        delta1=...
            repmat([points(1,frame-1,pid1,1) points(1,frame-1,pid1,2)],points(1,frame,max_points_per_frame,4),1)-...
            squeeze(points(1,frame,1:points(1,frame,max_points_per_frame,4),1:2));
        delta1=sum(delta1.^2,2).^0.5;
        [delta1,k]=sort(delta1);
        pid1=k(1);
        if delta1(1)<pixel_tol && points(1,frame,k(1),4)==0
            ok1=1;
            lost1=0;
        else
            lost1=0;
        end
        %find close by points to pid2 since traj still too short
        delta2=...
            repmat([points(2,frame-1,pid2,1) points(2,frame-1,pid2,2)],points(2,frame,max_points_per_frame,4),1)-...
            squeeze(points(2,frame,1:points(2,frame,max_points_per_frame,4),1:2));
        delta2=sum(delta2.^2,2).^0.5;
        [delta2,k]=sort(delta2);
        pid2=k(1);
        if delta2(1)<pixel_tol && points(2,frame,k(1),4)==0
            ok2=1;
            lost2=0;
        else
            lost2=1;
        end
    else
        %next neighbor for comparison
        delta1=...
            repmat([points(1,frame-1,pid1,1) points(1,frame-1,pid1,2)],points(1,frame,max_points_per_frame,4),1)-...
            squeeze(points(1,frame,1:points(1,frame,max_points_per_frame,4),1:2));
        delta1=sum(delta1.^2,2).^0.5;
        [delta1,k]=sort(delta1);
        pid1_next=k(1);
        [pid1,ok1]=glue_traj_2d(traj,1); %go glue, if possible just fill in points to algorithm doesn't even notice : -)
        if ok1==1
            glued1=1;
            if pid1~=pid1_next
                lost1=1;
            else
                lost1=0;
            end
        else
            glued1=0;
            lost1=1;
        end
        %next neighbor for comparison
        delta2=...
            repmat([points(2,frame-1,pid2,1) points(2,frame-1,pid2,2)],points(2,frame,max_points_per_frame,4),1)-...
            squeeze(points(2,frame,1:points(2,frame,max_points_per_frame,4),1:2));
        delta2=sum(delta2.^2,2).^0.5;
        [delta2,k]=sort(delta2);
        pid2_next=k(1);
        [pid2,ok2]=glue_traj_2d(traj,2); %go glue, if possible just fill in points to algorithm doesn't even notice : -)
        if ok2==1
            glued2=1;
            if pid2~=pid2_next
                lost2=1;
            else
                lost2=0;
            end
        else
            glued2=0;
            lost2=1;
        end
    end
    
    dist=-1;
    delta=-1;
    if ok1==1 & ok2==1
        %get 3d pos
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        [points3D,dist]=get_3d_point_direct(frame,pid1,pid2);%%%%%%%%%%%%%%%%%%%%%%%%%%% remove diect for ptv 2cam matlab v3
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        delta=sum((points3D-traj(end,4:6)).^2,2)^0.5;
        
        %update traj if point ok from matching AND from tracking
        
        %%%%should be more tolerant if very small distances in image space,
        %%%%i.e. if very slow velocity anyway....
        
        if dist<tol & delta<max(track_tol,tol) %& (points3D(1,1)^2+points3D(1,3)^2)^0.5<domain_radius
            
            if points(1,frame,pid1,5)==1 | points(2,frame,pid2,5)==1
                glued=1;
            else
                glued=0;
            end
            traj=[traj;frame pid1 pid2 points3D(1,1:3) dist delta*fps glued];
        else
            ok=0;
        end
        if frame>=num_frames-1
            ok=0;
        end
    else
        ok=0;
        aa=1;
    end
end
if ok==0
    %look why it failed
%     tx=['length: ',num2str(length(traj)),', lost1: ',num2str(lost1),', glued1: ',num2str(glued1),', lost2: ',num2str(lost2),', glued2: ',num2str(glued2),', ray-mismatch: ',num2str((dist-tol)*1000),', vel-mismatch: ',num2str((delta-max(track_tol,tol))*1000)];
%     disp(tx)
    si=size(traj);
    reason(1)=si(1,1);
    reason(2)=lost1;
    reason(3)=glued1;
    reason(4)=lost2;
    reason(5)=glued2;
    reason(6)=(dist-tol)*1000;
    reason(7)=(delta-max(track_tol,tol))*1000;
    aa=1;
end
aa=1;

