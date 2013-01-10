function calc_pair_dispersion(name_dispersion)

global final_traj num_frames fps
global r0 radius_constraint x_constraint y_constraint


si=size(final_traj);
if exist(name_dispersion)%1>2%
    load(name_dispersion)
    first=saved_i;
else
    pc=0;
    le_co=zeros(2*si(1,1),1);
    r_stat=zeros(num_frames,7);% 1=count,2=r,3=rx,4=ry,5=rz,6=rhor,7=rver
    first=1;
end

dif=final_traj(2:si(1,1),1)-final_traj(1:si(1,1)-1,1);
dif=[dif(1);dif];
beg_ind=find(dif<0);
end_ind=beg_ind-1;
end_ind=[end_ind;si(1,1)];
beg_ind=[1;beg_ind];

for i=first:si(1,1)
    if mod(i,100)==0
        %[i pc mean(le_co(1:pc))]
        tx0=['processed points: ',num2str(i)];
        tx1=['completed:....... ',num2str(round(i/si(1,1)*100)),'%'];
        tx2=['points with pair: ',num2str(round(pc/i*100)),'%'];
        tx3=['mean pair length: ',num2str(mean(le_co(1:pc)))];
        tx4=['--------------------------------------'];
        disp(tx0);
        disp(tx1);
        disp(tx2);
        disp(tx3);
        disp(tx4);
    end
    if mod(i,10000)==0
        saved_i=i;
        save(name_dispersion,'saved_i','pc','le_co','r_stat');
    end
    time=final_traj(i,1);
    %find candidates of same time
    ind_t=find(final_traj(:,1)==time);
    % find candidates of same time and close 'enough'
    delta=...
        repmat([final_traj(i,6) -final_traj(i,8) final_traj(i,7)],length(ind_t),1)-...
        [final_traj(ind_t,6) -final_traj(ind_t,8) final_traj(ind_t,7)];
    delta=sum(delta.^2,2).^0.5;
    [delta,k]=sort(delta);
    ind_d=find(delta<r0);
    ind_d(1)=[];%remove the point itself, its distance is zero.
    if length(ind_d)>0
        ind=ind_t(k(ind_d));
    else
        ind=[];
    end
    if length(ind)>0
       %we have a pair!!
       for p=1:length(ind)
          %calc pair dispersion stats
          
          %find how much is left of traj starting at i
          en_i=find_end_of_traj_v2(i,beg_ind,end_ind);
          %check how much of it remains inside our circel r=0.025 @x=0,y=0.025
          en_i=find_points_inside_circle(i,en_i,radius_constraint,x_constraint,y_constraint);%-1 if nothing
          %find how much is left of traj starting at ind(p)
          en_ind=find_end_of_traj_v2(ind(p),beg_ind,end_ind);
          %check how much of it remains inside our circel r=0.025 @x=0,y=0.025
          en_ind=find_points_inside_circle(ind(p),en_ind,radius_constraint,x_constraint,y_constraint);%-1 if nothing
          if en_i>-1 & en_ind>-1
              pc=pc+1;
              if pc>si(1,1)
                  disp('aloccate more memory for pairs!!')
              end
              %determine common path
              le_i   = en_i-i+1;
              le_ind = en_ind-ind(p)+1;
              le_co(pc)=min(le_i,le_ind);
              %this is the evoluting separation vector
              r_vec=...
                  [final_traj(i     :i     +le_co(pc)-1,6) -final_traj(i     :i     +le_co(pc)-1,8) final_traj(i     :i     +le_co(pc)-1,7)]-...
                  [final_traj(ind(p):ind(p)+le_co(pc)-1,6) -final_traj(ind(p):ind(p)+le_co(pc)-1,8) final_traj(ind(p):ind(p)+le_co(pc)-1,7)];
              
              r=sum(r_vec.^2,2).^0.5;
              r_x=r_vec(:,1);
              r_y=r_vec(:,2);
              r_z=r_vec(:,3);
              r_hor=(r_vec(:,1).^2+r_vec(:,2).^2).^0.5;
              r_ver=r_vec(:,3);
              %and now we should put it into separation bins,
              %r,r_x,r_y,r_z,r_hor,r_ver
              r_stat(1:le_co(pc),1)=r_stat(1:le_co(pc),1)+repmat(1,le_co(pc),1);
              r_stat(1:le_co(pc),2)=r_stat(1:le_co(pc),2)+r.^2;
              r_stat(1:le_co(pc),3)=r_stat(1:le_co(pc),3)+r_x.^2;
              r_stat(1:le_co(pc),4)=r_stat(1:le_co(pc),4)+r_y.^2;
              r_stat(1:le_co(pc),5)=r_stat(1:le_co(pc),5)+r_z.^2;
              r_stat(1:le_co(pc),6)=r_stat(1:le_co(pc),6)+r_hor.^2;
              r_stat(1:le_co(pc),7)=r_stat(1:le_co(pc),7)+r_ver.^2;
          end
       end
    end
end
%take average for r_stat
maxi=1;
for i=1:num_frames
   if r_stat(i,1)>100
       r_stat_m(i,2)=r_stat(i,2)/r_stat(i,1);
       r_stat_m(i,3)=r_stat(i,3)/r_stat(i,1);
       r_stat_m(i,4)=r_stat(i,4)/r_stat(i,1);
       r_stat_m(i,5)=r_stat(i,5)/r_stat(i,1);
       r_stat_m(i,6)=r_stat(i,6)/r_stat(i,1);
       r_stat_m(i,7)=r_stat(i,7)/r_stat(i,1);
       if i>maxi
           maxi=i;
       end
   else
       r_stat_m(i,2:7)=0;
   end
end
t=1:num_frames;
t=(t-1)/fps;

figure;hold on;box on;
plot(t(1:maxi),r_stat(1:maxi,1));
set(gca,'XScale','log')
set(gca,'YScale','log')
xlabel('t (s)')
ylabel('# pairs')
tx=['r0=',num2str(r0),', r_c=',num2str(radius_constraint),', x_c=',num2str(x_constraint),', y_c=',num2str(y_constraint)];
title(tx);


figure;hold on; box on;
plot(t(1:maxi),r_stat_m(1:maxi,2).^0.5,'k');
plot(t(1:maxi),r_stat_m(1:maxi,3).^0.5,'r');
plot(t(1:maxi),r_stat_m(1:maxi,4).^0.5,'g');
plot(t(1:maxi),r_stat_m(1:maxi,5).^0.5,'b');
plot(t(1:maxi),r_stat_m(1:maxi,6).^0.5,'m');
plot(t(1:maxi),r_stat_m(1:maxi,7).^0.5,'c');
xlabel('t (s)')
ylabel('\Delta (m)')
set(gca,'XScale','log')
set(gca,'YScale','log')
legend('r','r_x','r_y','r_z','r_{hor}','r_{ver}','Location','NorthWest')
tx=['r0=',num2str(r0),', r_c=',num2str(radius_constraint),', x_c=',num2str(x_constraint),', y_c=',num2str(y_constraint)];
title(tx);

figure;hold on; box on;
plot(t(1:maxi),r_stat_m(1:maxi,2),'k');
plot(t(1:maxi),r_stat_m(1:maxi,3),'r');
plot(t(1:maxi),r_stat_m(1:maxi,4),'g');
plot(t(1:maxi),r_stat_m(1:maxi,5),'b');
plot(t(1:maxi),r_stat_m(1:maxi,6),'m');
plot(t(1:maxi),r_stat_m(1:maxi,7),'c');
xlabel('t (s)')
ylabel('\Delta^2 (m^2)')
set(gca,'XScale','log')
set(gca,'YScale','log')
legend('r^2','r_x^2','r_y^2','r_z^2','r_{hor}^2','r_{ver}^2','Location','NorthWest')
tx=['r0=',num2str(r0),', r_c=',num2str(radius_constraint),', x_c=',num2str(x_constraint),', y_c=',num2str(y_constraint)];
title(tx);


pc=pc