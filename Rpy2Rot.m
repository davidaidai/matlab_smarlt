function R = Rpy2Rot(x,y,z)
%%
% Input parameters: angles of roll(x),pitch(y) and yaw(z)
% Output parameters: Rotary matrix
% x = Deg2Rad(x);
% y = Deg2Rad(y);
% z = Deg2Rad(z);
cx = cos(x);
sx = sin(x);
cy = cos(y);
sy = sin(y);
cz = cos(z);
sz = sin(z);
R = [cz*cy,-sz*cx+cz*sy*sx,sz*sx+cz*sy*cx
     sz*cy,cz*cx+sz*sy*sx,-cz*sx+sz*sy*cx
     -sy,cy*sx,cy*cx];