function be_i=find_end_of_traj_v2(i,beg_ind,end_ind)

ind=find(i-end_ind<=0 & i-beg_ind>=0);
be_i=end_ind(ind);