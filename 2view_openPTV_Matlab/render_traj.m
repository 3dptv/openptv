function render_traj(min_length)

global final_traj

si=size(final_traj);

dif=final_traj(2:si(1,1),1)-final_traj(1:si(1,1)-1,1);
dif=[dif(1);dif];
beg_ind=find(dif<0);
end_ind=beg_ind-1;
end_ind=[end_ind;si(1,1)];
beg_ind=[1;beg_ind];
traj_length=end_ind-beg_ind;

good_traj_ind=find(traj_length>=min_length);
good_ind=[];
for i=1:length(good_traj_ind)
    good_ind=[good_ind;(beg_ind(good_traj_ind(i)):end_ind(good_traj_ind(i)))'];
end

figure(30);
velo=(final_traj(good_ind,9).^2+final_traj(good_ind,10).^2+final_traj(good_ind,11).^2).^0.5;
scatter3(final_traj(good_ind,6),-final_traj(good_ind,8),final_traj(good_ind,7),15,velo,'filled');
xlabel('x');
ylabel('-z');
zlabel('y');
box on;
axis equal;
colorbar

[length(traj_length) length(good_traj_ind)]

