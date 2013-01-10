% image_name=['C:\Users\Luethi\Documents\Dropbox\PTV_working_folders\JMC_Oct_26\cal1/cal1.tif'];
% image=imread(image_name);
% image1 = image(1:1024,1:512);
% image2 = image(1:1024,513:1024);
% name1=['C:\Users\Luethi\Documents\Dropbox\PTV_working_folders\JMC_Oct_26\cal1/cal1_cam1.tif'];
% name2=['C:\Users\Luethi\Documents\Dropbox\PTV_working_folders\JMC_Oct_26\cal1/cal1_cam2.tif'];
% imwrite(image1,name1,'tif');
% imwrite(image2,name2,'tif');

%preliminaries, compute average image that will be subtracted
first=1;
last=2000;

for i=first:last
    if mod(i,20)==0
        [1 i]
    end
    image_name=['C:\Users\Luethi\Documents\Photron\Photron FASTCAM Viewer 3\JMC_Oct26\2000fps_14ml_2Hz_180deg_rec1_C001H001S0001/2000fps_14ml_2Hz_180deg_rec1_C001H001S000',num2str(1000000+i),'.tif'];
    image=double(imread(image_name));
    if i==first
        A=image;
    else
       A=A+image; 
    end
end
A=A/(last-first+1);
mini=min(min(A));
maxi=max(max(A));
figure;h=surf(A);set(h,'EdgeColor','none');view([0 0 1]);
%end of preliminaries

for i=first:last
    if mod(i,20)==0
        [2 i]
    end
    image_name=['C:\Users\Luethi\Documents\Photron\Photron FASTCAM Viewer 3\JMC_Oct26\2000fps_14ml_2Hz_180deg_rec1_C001H001S0001/2000fps_14ml_2Hz_180deg_rec1_C001H001S000',num2str(1000000+i),'.tif'];
    image=imread(image_name);
    image1 = double(image(1:1024,1  :512 ))-A(1:1024,1  :512 );
    ind=find(image1<0);
    image1(ind)=0;
    image1=uint8(image1);
    image2 = double(image(1:1024,513:1024))-A(1:1024,513:1024);
    ind=find(image2<0);
    image2(ind)=0;
    image2=uint8(image2);
    name1=['C:\Users\Luethi\Documents\Dropbox\PTV_working_folders\JMC_Oct_26\img_2Hz_rec1/2000fps_2Hz_rec1_cam1.',num2str(i-first+1)];
    name2=['C:\Users\Luethi\Documents\Dropbox\PTV_working_folders\JMC_Oct_26\img_2Hz_rec1/2000fps_2Hz_rec1_cam2.',num2str(i-first+1)];
    imwrite(image1,name1,'tif');
    imwrite(image2,name2,'tif');
end
