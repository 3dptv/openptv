function [points3D,dist]=intersectRays(A,B,U,V)


% taken from: Distance between 2 lines in 3d space
% http://groups.google.pl/group/comp.soft-sys.matlab/browse_thread/thread/a81803de728c9684/602e6bbf4c755565?hl=pl#602e6bbf4c755565
si=size(U);
for i=1:si(1,1)
    W = cross(U(i,:),V(i,:));
    P1 = A + dot(cross(B-A,V(i,:)),W)/dot(W,W)*U(i,:);
    P2 = B + dot(cross(B-A,U(i,:)),W)/dot(W,W)*V(i,:);
    
    dist(i) = norm(P2-P1);
    points3D(i,1:3)=0.5*(P1+P2);
end


