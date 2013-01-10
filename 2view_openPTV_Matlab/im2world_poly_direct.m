function dir=im2world_poly_direct(x,y,cam)


global im2world_const_v2
global poly_order

si=size(x);

% linear
A=[repmat(1,si(1,1),1) x y];
% add quadratic terms
if poly_order>1
    A=[A x.^2 y.^2 x.*y];
end
% add cubic terms
if poly_order>2
    A=[A x.^3 y.^3 x.^2.*y y.^2.*x];
end


Y=A*im2world_const_v2{cam};
dir=Y;


