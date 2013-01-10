function [pid,ok]=glue_traj_2d(traj,cam)

global points
global num_frames
global max_points_per_frame
global dir_horizontal
global center_point
global dir_vertical
global pos
global pix
global accuracy_2d
global point_proj


si=size(traj);
le=si(1,1);
%determine order, which is used to attempt gluing jump
order=2;
max_jump=30;
fit_range=30;

%prepare polynomial fit for jumps
en=le;
be=en-fit_range;
if be<1
    be=1;
end

for i=be:le
    trax(i-be+1)=points(cam,traj(i,1),traj(i,cam+1),1);
    tray(i-be+1)=points(cam,traj(i,1),traj(i,cam+1),2);
end

A=zeros(en-be+1,order+1);
y=zeros(en-be+1,2);
count=0;
for frame=be:en
    count=count+1;
    time=traj(frame,1);
    for or=0:order
        A(count,or+1)=time^or;
    end
    y(count,1:2)=[trax(frame-be+1) tray(frame-be+1)];
end
if det(A'*A)>1e-15
    X=(A'*A)\A'*y;
    nogo=0;
else
    nogo=1;
end

%loop through jump size until max jump is reached
last_frame=traj(end,1);
pid=-1;
ok=0;
if nogo==0
    for jump=1:max_jump
        %jump and check in n-dim space whether anything is
        %close enough or not, i.e. x,u?,size?
        time=last_frame+jump;
        if time>num_frames
            pid=-1;
            ok=0;
            break;
        end
        %howmany points in frame n?
        num_part=points(cam,time,max_points_per_frame,4);
        proj=[];
        for n=1:2
            proj(n)=0;
            for or=0:order
                proj(n)=proj(n)+X(or+1,n)*time^or;
            end
        end
        proj=repmat(proj,num_part,1);
        if num_part>1
            delta=proj-squeeze(points(cam,time,1:num_part,1:2));
        else %%%fix for unwanted transposed
            delta=proj-squeeze(points(cam,time,1:num_part,1:2)');
        end
        %exclude those canditates, which are already part of another trajectory
        delta=sum(delta.^2,2).^0.5;
        [delta,k]=sort(delta);
        
        if delta(1)<accuracy_2d %%%&& points(cam,time,k(1),4)==0
            %create points in-betwen and then make pid for time=0!!!!
            if jump==1
                pid=k(1);
                ok=1;
                if cam==1
                    point_proj(1)=points(cam,time,pid,1);
                    point_proj(2)=pix(2)-points(cam,time,pid,2)+1;
                end
                break;
            else
                for i=1:jump-1
                    time=last_frame+i;
                    proj=[];
                    for n=1:2
                        proj(n)=0;
                        for or=0:order
                            proj(n)=proj(n)+X(or+1,n)*time^or;
                        end
                    end
                    points(cam,time,max_points_per_frame,4)=points(cam,time,max_points_per_frame,4)+1;
                    pid=points(cam,time,max_points_per_frame,4);
                    points(cam,time,pid,1)=proj(1);
                    points(cam,time,pid,2)=proj(2);
                    
                    dir=im2world_poly_direct(points(cam,time,pid,1),points(cam,time,pid,2),1);
                    [point,t]=intersectionPlaneLine(pos(cam,1:3),dir_horizontal,dir,center_point,dir_vertical);
                    points(cam,frame,pid,3)=t;
                    
                    points(cam,time,pid,5)=1; %now it is marked asproduced from gluwing!!
                end
                %%%now we assign actual pid that will help to continue the show
                pid=points(cam,last_frame+1,max_points_per_frame,4);
                ok=1;
                if cam==1
                    point_proj(1)=points(cam,time,pid,1);
                    point_proj(2)=pix(2)-points(cam,time,pid,2)+1;
                end
                break; %%breaks of for loop through jump
            end
        end
    end
else
    pid=-1;
    ok=0;
end



aa=1;