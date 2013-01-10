function calc_single_dispersion_hopping(name_dispersion)

global final_traj num_frames fps
global radius_constraint x_constraint y_constraint baffle_id

max_search=-20;
hopp_tol=10e-3;

si=size(final_traj);
if exist(name_dispersion)%1>2%
    load(name_dispersion)
    first=saved_i;
else
    sc=0;
    len_si=zeros(si(1,1),1);
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
    if mod(i,10)==0
        %[i pc mean(le_si(1:sc))]
        tx0=['processed points: ',num2str(i)];
        tx1=['completed:....... ',num2str(round(i/si(1,1)*100)),'%'];
        tx2=['mean traj length: ',num2str(mean(len_si(1:sc)))];
        tx3=['--------------------------------------'];
        disp(tx0);
        disp(tx1);
        disp(tx2);
        disp(tx3);
    end
    if mod(i,10000)==0 & i>100
        saved_i=i;
        save(name_dispersion,'saved_i','sc','len_si','r_stat');
    end
    
    %calc single dispersion stats
    
    %find how much is left of traj starting at i
    en_i=find_end_of_traj_v2(i,beg_ind,end_ind);
    %check how much of it remains inside our circel r=0.025 @x=0,y=0.025
    en_i=find_points_inside_circle(i,en_i,radius_constraint,x_constraint,y_constraint);%-1 if nothing
    le_si=en_i-i+1;
    single=[];
    
    if le_si>-max_search
        % ok, let's go, make copy of what we have sofar
        single=[final_traj(i:i+le_si-1,1) final_traj(i:i+le_si-1,6) -final_traj(i:i+le_si-1,8) final_traj(i:i+le_si-1,7)];
        hopp=1;
        while hopp==1
            hopp=0;
            for search=0:-1:max_search
                % check if there is something close by to hopp, check backwards for some length
                if length(single)+search>0
                    en_sin_ind=length(single);
                    time=single(en_sin_ind+search,1);
                    cand_ind=find(final_traj(:,1)==time & baffle_id==baffle_id(en_sin_ind+search));%to make sure that hopp stays in same chamber!
                    if length(cand_ind)>0
                        pos=repmat(single(en_sin_ind+search,2:4),length(cand_ind),1);
                        delta=pos-[final_traj(cand_ind,6) -final_traj(cand_ind,8) final_traj(cand_ind,7)];
                        delta=sum(delta.^2,2).^0.5;
                        [delta,k]=sort(delta);
                        ind=find(delta<hopp_tol);
                        if length(ind)>1 
                            %go through all possible hopps, starting from
                            %the closest one, but not itself, hence
                            %ii=2:...to see if it helps, i.e. if the gain
                            %is long enough
                            for ii=2:length(ind)
                                % find en_i etc
                                iii=cand_ind(k(ii));
                                en_i=find_end_of_traj_v2(iii,beg_ind,end_ind);
                                en_i=find_points_inside_circle(iii,en_i,radius_constraint,x_constraint,y_constraint);%-1 if nothing
                                le_si=en_i-iii+1;
                                if le_si>=-search-max_search %<--here we check if the gain is long enough, i.e. worthwhile
                                    %now append new traj seg to existing 'single'
                                    single=[single(1:en_sin_ind+search-1,:);[final_traj(iii:iii+le_si-1,1) final_traj(iii:iii+le_si-1,6) -final_traj(iii:iii+le_si-1,8) final_traj(iii:iii+le_si-1,7)]];
                                    hopp=1;
                                    break;
                                end
                            end
                            if hopp==1
                                break;
                            end
                        end
                    end
                end
            end
        end
    end
    
    if length(single)>0
        sc=sc+1;
        if sc>si(1,1)
            disp('aloccate more memory for singles!!')
        end
        si_si=size(single);
        len_si(sc)=si_si(1,1);
        %this is the evoluting separation vector
        r_vec=single(:,2:4)-repmat(single(1,2:4),len_si(sc),1);
        
        r=sum(r_vec.^2,2).^0.5;
        r_x=r_vec(:,1);
        r_y=r_vec(:,2);
        r_z=r_vec(:,3);
        r_hor=(r_vec(:,1).^2+r_vec(:,2).^2).^0.5;
        r_ver=r_vec(:,3);
        %and now we should put it into separation bins,
        %r,r_x,r_y,r_z,r_hor,r_ver
        r_stat(1:len_si(sc),1)=r_stat(1:len_si(sc),1)+repmat(1,len_si(sc),1);
        r_stat(1:len_si(sc),2)=r_stat(1:len_si(sc),2)+r.^2;
        r_stat(1:len_si(sc),3)=r_stat(1:len_si(sc),3)+r_x.^2;
        r_stat(1:len_si(sc),4)=r_stat(1:len_si(sc),4)+r_y.^2;
        r_stat(1:len_si(sc),5)=r_stat(1:len_si(sc),5)+r_z.^2;
        r_stat(1:len_si(sc),6)=r_stat(1:len_si(sc),6)+r_hor.^2;
        r_stat(1:len_si(sc),7)=r_stat(1:len_si(sc),7)+r_ver.^2;
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
ylabel('# single trajectories')
tx=['r_c=',num2str(radius_constraint),', x_c=',num2str(x_constraint),', y_c=',num2str(y_constraint)];
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
tx=['r_c=',num2str(radius_constraint),', x_c=',num2str(x_constraint),', y_c=',num2str(y_constraint)];
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
tx=['r_c=',num2str(radius_constraint),', x_c=',num2str(x_constraint),', y_c=',num2str(y_constraint)];
title(tx);


figure;hold on; box on;
nhist(len_si(1:sc),100);
xlabel('length of hopped trajectories')
ylabel('pdf')
set(gca,'XScale','log')
set(gca,'YScale','log')


sc=sc