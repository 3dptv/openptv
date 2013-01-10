 function en_inside=find_points_inside_circle(be,en,radius,x,y)
 
 
 global final_traj
 
 en_inside=-1;
 for i=be:en
     dist=((final_traj(i,6)-x)^2+(-final_traj(i,8)-y)^2)^0.5;
     if dist<radius
         en_inside=i;
     else
         break;
     end
 end
 aa=1;