function render_traj_figures(render_level,al0,phase_shift,amp)

global num_frames fps Hz
global final_traj
global baffle_y_pos domain_radius hole_radius holes

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
    caxis([0 0.1])
    colorbar
    %draw_baffles(30,baffle_y_pos,domain_radius,hole_radius,holes)
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


% figure(42);hold on;
% scatter(final_traj(:,6),final_traj(:,11),1,'filled'); %vz versus x
% xlabel('x');
% ylabel('vz');
% scatter(final_traj(:,8),final_traj(:,9),1,'r','filled'); %vx versus z
% xlabel('x, z');
% ylabel('vz, vx');

for i=1:num_frames
   ind=find(final_traj(:,1)==i);
   av_vert(i)  = mean(final_traj(ind,10));
   rms_vert(i) = mean(final_traj(ind,10).^2)^0.5;
   rms_rad(i)  = mean((final_traj(ind,9).^2+final_traj(ind,11).^2).^0.5);
end

%find phase angles of entire flow
figure(42);hold on;
al=[1:num_frames]/fps*360*Hz;
%al0=320,%246;%20;%152;%215;%
%phase_shift=40;
al1=al0+phase_shift;%30
cos_fit=amp*cosd(al-al0);
plot(al,av_vert,'r')
plot(al,cos_fit,'m')
plot(al,rms_vert,'g')
plot(al,rms_rad,'b')
line([al0 al0],[-0.2 0.5],'Color','r')
line([al0-180 al0-180],[-0.2 0.5],'Color','r')
line([al0+180 al0+180],[-0.2 0.5],'Color','r')
line([al0+2*180 al0+2*180],[-0.2 0.5],'Color','r')
line([al1 al1],[-0.2 0.5],'Color','b')
line([al1-180 al1-180],[-0.2 0.5],'Color','b')
line([al1+180 al1+180],[-0.2 0.5],'Color','b')
line([al1+2*180 al1+2*180],[-0.2 0.5],'Color','b')
xlabel('\alpha (deg)')
xlim([0 num_frames/fps*Hz*360])
%ylim([-0.1 0.2])
box on;
legend('av vert','fit av vert','rms vert','rms rad')
tx=[num2str(Hz),' Hz, phase shift: ',num2str(phase_shift)];
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