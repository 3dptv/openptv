function glue_all_traj_3d()

global final_traj num_frames accuracy_3d

si=size(final_traj);
dif=final_traj(2:si(1,1),1)-final_traj(1:si(1,1)-1,1);
dif=[dif(1);dif];
beg_ind=find(dif<0);
end_ind=beg_ind-1;
end_ind=[end_ind;si(1,1)];
beg_ind=[1;beg_ind];
seg_length=end_ind-beg_ind+1;

%determine order, which is used to attempt gluing jump
order=2;
max_jump=30;
fit_range=30;

all_end_processed=0;
e=0;
[e length(end_ind) round(mean(seg_length))]
while all_end_processed==0 & e<length(end_ind)
    e=e+1;
    if mod(e,50)==0
        [e length(end_ind) round(mean(seg_length))]
    end
    %have a look if traj doesn't continue
    
    %prepare polynomial fit for jumps
    en=end_ind(e);
    be=en-fit_range;
    if be<beg_ind(e)
        be=beg_ind(e);
    end
    for i=be:en
        trax(i-be+1)= final_traj(i,6);
        tray(i-be+1)= final_traj(i,7);
        traz(i-be+1)= final_traj(i,8);
    end
    
    A=zeros(en-be+1,order+1);
    y=zeros(en-be+1,2);
    count=0;
    for point_id=be:en
        count=count+1;
        time=final_traj(point_id,1);
        for or=0:order
            A(count,or+1)=time^or;
        end
        y(count,1:3)=[trax(point_id-be+1) tray(point_id-be+1) traz(point_id-be+1)];
    end
    if det(A'*A)>1e-15
        X=(A'*A)\A'*y;
        nogo=0;
    else
        nogo=1;
    end
    
    %loop through jump size until max jump is reached
    last_frame=final_traj(end_ind(e),1);
    if nogo==0
        for jump=1:max_jump
            %jump and check in n-dim space whether anything is
            %close enough or not, i.e. x,u?,size?
            time=last_frame+jump;
            if time>num_frames
                break;
            end
            %howmany starting trajectories in frame n? 
            tmp=find(final_traj(beg_ind,1)==time);
            ind_beg_t=beg_ind(tmp);
            ind_end_t=end_ind(tmp);
            num_part=length(ind_beg_t);
            if num_part>0
                proj=[];
                for n=1:3
                    proj(n)=0;
                    for or=0:order
                        proj(n)=proj(n)+X(or+1,n)*time^or;
                    end
                end
                proj=repmat(proj,num_part,1);
                delta=proj-[final_traj(ind_beg_t,6) final_traj(ind_beg_t,7) final_traj(ind_beg_t,8)];
                %exclude those canditates, which are already part of another trajectory
                delta=sum(delta.^2,2).^0.5;
                [delta,k]=sort(delta);
                
                if delta(1)<accuracy_3d
                    %create points in-betwen and then make pid for time=0!!!!
                    if jump==1
                        remains=final_traj;
                        cut=final_traj(ind_beg_t(k(1)):ind_end_t(k(1)),1:12);
                        remains(ind_beg_t(k(1)):ind_end_t(k(1)),:)=[];
                        remains=remains(end_ind(e)+1:end,1:12);
                        final_traj=[final_traj(1:end_ind(e),1:12);cut;remains];
                        
                        %now we badly need an update...
                        si=size(final_traj);
                        dif=final_traj(2:si(1,1),1)-final_traj(1:si(1,1)-1,1);
                        dif=[dif(1);dif];
                        beg_ind=find(dif<0);
                        end_ind=beg_ind-1;
                        end_ind=[end_ind;si(1,1)];
                        beg_ind=[1;beg_ind];
                        seg_length=end_ind-beg_ind+1;
                        e=e-1;
                        
                        break;
                    else
                        %update A and y for proper landing
                        be=ind_beg_t(k(1));
                        en=be+fit_range;
                        if en>ind_end_t(k(1))
                            en=ind_end_t(k(1));
                        end
                        for i=be:en
                            trax(i-be+1)= final_traj(i,6);
                            tray(i-be+1)= final_traj(i,7);
                            traz(i-be+1)= final_traj(i,8);
                        end

                        for point_id=be:en
                            count=count+1;
                            time=final_traj(point_id,1);
                            for or=0:order
                                A(count,or+1)=time^or;
                            end
                            y(count,1:3)=[trax(point_id-be+1) tray(point_id-be+1) traz(point_id-be+1)];
                        end
                        X=(A'*A)\A'*y;
                        %end of A y update
                        new_seg=[];
                        for i=1:jump-1
                            time=last_frame+i;
                            proj=[];
                            for n=1:3
                                proj(n)=0;
                                for or=0:order
                                    proj(n)=proj(n)+X(or+1,n)*time^or;
                                end
                            end
                            new_seg=[new_seg;time 0 0 0 0 proj(1) proj(2) proj(3) 0 0 0 0];
                        end
                        
                        remains=final_traj;
                        cut=final_traj(ind_beg_t(k(1)):ind_end_t(k(1)),1:12);
                        remains(ind_beg_t(k(1)):ind_end_t(k(1)),:)=[];
                        remains=remains(end_ind(e)+1:end,1:12);
                        final_traj=[final_traj(1:end_ind(e),1:12);new_seg;cut;remains];
                        
                        %now we badly need an update...
                        si=size(final_traj);
                        dif=final_traj(2:si(1,1),1)-final_traj(1:si(1,1)-1,1);
                        dif=[dif(1);dif];
                        beg_ind=find(dif<0);
                        end_ind=beg_ind-1;
                        end_ind=[end_ind;si(1,1)];
                        beg_ind=[1;beg_ind];
                        seg_length=end_ind-beg_ind+1;
                        e=e-1;
                        
                        break; %%breaks of for loop through jump
                    end
                end
            end
        end
    end
end

aa=1;