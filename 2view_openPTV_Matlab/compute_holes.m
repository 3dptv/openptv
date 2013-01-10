function compute_holes()

global baffle_y_pos domain_radius holes final_traj

[x_odd,x_even]=calc_baffle_holes(20,baffle_y_pos,domain_radius);
%find points close to baffle, potentially such close to baffle
%holes: vz should dominate and should be close to baffle
holes=zeros(6,length(x_odd),2);
offset=[0 0];
for baff_id=1:6
    if mod(baff_id,2)==1
        holes(baff_id,:,:)=x_odd+repmat(offset,length(x_odd),1);
        col='g';
    else
        holes(baff_id,:,:)=x_even+repmat(offset,length(x_odd),1);
        col='b';
    end
    baff=repmat(baffle_y_pos(baff_id),length(final_traj),1);
    ypos=repmat(final_traj(:,7),1,length(baffle_y_pos(baff_id)));
    diff=(abs(ypos-baff)')';
    %find points close to baffles and with dominating vz
    ind=find(diff<2e-3 & abs(final_traj(:,10))>2*abs(final_traj(:,9)) & abs(final_traj(:,10))>2*abs(final_traj(:,11))) ;% ;
    figure(100+baff_id);hold on;
    scatter(final_traj(ind,6),-final_traj(ind,8),5,col,'filled');
    scatter(holes(baff_id,:,1),holes(baff_id,:,2),800,'r');
    xlabel('x');
    ylabel('-z');
    tx=['baffle id: ',num2str(baff_id)];
    title(tx)
    box on;
    axis equal;
end

aa=1;