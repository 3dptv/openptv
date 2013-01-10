function render_traj_figures_v2(render_level)

global num_frames fps Hz
global final_traj

if render_level>0
    disp('busy with the main trajectory and baffles figure')
    figure(30);
    velo=(final_traj(:,9).^2+final_traj(:,10).^2+final_traj(:,11).^2).^0.5;
    glue=(final_traj(:,12));
    scatter3(final_traj(:,6),-final_traj(:,8),final_traj(:,7),15,velo,'filled');
    %scatter3(final_traj(:,6),-final_traj(:,8),final_traj(:,7),5,glue,'filled');
    xlabel('x');
    ylabel('-z');
    zlabel('y');
    box on;
    axis equal;
    caxis([0 0.6])
    colorbar
    disp('done with heavy figure')
end

figure(40);
[nout,xout]=nhist(final_traj(:,4),100);
plot(xout,nout)
xlabel('dist rays [m]')
ylabel('pdf')

figure(41);hold off;
[nout,xout]=nhist(final_traj(:,5),1000);
plot(xout,nout,'k')
xlabel('raw vel [m/s]')
ylabel('pdf')
hold on;

[nout,xout]=nhist((final_traj(:,9).^2+final_traj(:,10).^2+final_traj(:,11).^2).^0.5,1000);
plot(xout,nout,'m')
xlabel('vel [m/s]')
ylabel('pdf')

[nout,xout]=nhist(final_traj(:,9),1000);
plot(xout,nout,'r')
xlabel('vel [m/s]')
ylabel('pdf')
[nout,xout]=nhist(final_traj(:,10),1000);
plot(xout,nout,'g')
xlabel('vel [m/s]')
ylabel('pdf')
[nout,xout]=nhist(final_traj(:,11),1000);
plot(xout,nout,'b')
xlabel('vel [m/s]')
ylabel('pdf')
box on;
legend('raw vel','filt vel','vx','vy','vz')
set(gca,'YScale','log')

figure(46);
[nout,xout]=nhist(traj_length_stat(1:traj_count),100);
loglog(xout,nout);
xlabel('traj length (frames)');
ylabel('pdf');
tx=['num traj: ',num2str(traj_count)];
title(tx);

figure(47);
[nout,xout]=nhist(dist_travelled(1:traj_count),100);
loglog(xout,nout.*xout);
xlabel('mean dist travelled (m)');
ylabel('pdf\cdotx');
tx=['num traj: ',num2str(traj_count)];
title(tx);


if render_level>1
    figure(43);
    scatter(final_traj(:,9),final_traj(:,11),1,'filled'); %vx versus vz
    xlabel('vx');
    ylabel('vz');
    axis equal
    box on;
    
    figure(44);
    scatter(final_traj(:,9),final_traj(:,10),1,'filled'); %vx versus vz
    xlabel('vx');
    ylabel('vy');
    axis equal
    box on;
    
    figure(45);
    scatter(final_traj(:,11),final_traj(:,10),1,'filled'); %vx versus vz
    xlabel('vz');
    ylabel('vy');
    axis equal
    box on;
end