function calc_baffle_id(name_baffle)

global final_traj baffle_y_pos

si=size(final_traj);

baffle_id=zeros(si(1,1),1);
for i=1:si(1,1)
    ind=find(final_traj(i,7)-baffle_y_pos>0);
    if length(ind)>0
        baffle_id(i)=ind(end);
    else
        baffle_id(i)=-1;
    end
end

save(name_baffle,'baffle_id');