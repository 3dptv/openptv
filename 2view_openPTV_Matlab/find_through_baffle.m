function hol_pos=find_through_baffle(baff_id)

global holes
global final_traj
global domain_radius
global hole_radius
global baffle_y_pos

si=size(final_traj);

dif=final_traj(2:si(1,1),1)-final_traj(1:si(1,1)-1,1);
dif=[dif(1);dif];
beg_ind=find(dif<0);
end_ind=beg_ind-1;
end_ind=[end_ind;si(1,1)];
beg_ind=[1;beg_ind];

hol_pos=[];
c=0;
for i=1:length(beg_ind)
    diff=final_traj(beg_ind(i):end_ind(i),7)-baffle_y_pos(baff_id);%+0.0125;
    ra=sum((final_traj(beg_ind(i):end_ind(i),6).^2+final_traj(beg_ind(i):end_ind(i),8).^2),2).^0.5;
    minD=min(diff);
    maxD=max(diff);
    if minD*maxD<0 & ra<0.9*domain_radius
        %it goees accross
        [diff,k]=sort(abs(diff));
        c=c+1;
        hol_pos(c,1:3)=[final_traj(beg_ind(i)+k(1)-1,6) -final_traj(beg_ind(i)+k(1)-1,8) final_traj(beg_ind(i)+k(1)-1,7)];
    end
end


