function re_calc_vel()

global final_traj fps

kernel_width=20;
golay_order=2;

si=size(final_traj);
dif=final_traj(2:si(1,1),1)-final_traj(1:si(1,1)-1,1);
dif=[dif(1);dif];
beg_ind=find(dif<0);
end_ind=beg_ind-1;
end_ind=[end_ind;si(1,1)];
beg_ind=[1;beg_ind];
seg_length=end_ind-beg_ind+1;

for i=1:length(beg_ind)
    
    px = smooth(final_traj(beg_ind(i):end_ind(i),6),kernel_width,'sgolay',golay_order);
    py = smooth(final_traj(beg_ind(i):end_ind(i),7),kernel_width,'sgolay',golay_order);
    pz = smooth(final_traj(beg_ind(i):end_ind(i),8),kernel_width,'sgolay',golay_order);
    vx  = (px(2:end)-px(1:end-1))*fps;
    vy  = (py(2:end)-py(1:end-1))*fps;
    vz  = (pz(2:end)-pz(1:end-1))*fps;
    vx=[vx(1);vx];
    vy=[vy(1);vy];
    vz=[vz(1);vz];
    vx = smooth(vx,kernel_width,'sgolay',golay_order);
    vy = smooth(vy,kernel_width,'sgolay',golay_order);
    vz = smooth(vz,kernel_width,'sgolay',golay_order);
    final_traj(beg_ind(i):end_ind(i),6)=px;
    final_traj(beg_ind(i):end_ind(i),7)=py;
    final_traj(beg_ind(i):end_ind(i),8)=pz;
    final_traj(beg_ind(i):end_ind(i),9)=vx;
    final_traj(beg_ind(i):end_ind(i),10)=vy;
    final_traj(beg_ind(i):end_ind(i),11)=vz;
    
end
