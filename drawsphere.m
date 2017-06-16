function []=drawsphere(sphi,stheta,sr,center_p)
cpts=100;
phi=linspace(0,sphi,cpts);
theta=linspace(0,stheta,cpts);
[pp,tt]=meshgrid(phi,theta);
x=-sr*cos(pp).*sin(tt)+center_p(1)*ones(cpts,cpts);
y=sr*cos(pp).*cos(tt)+center_p(2)*ones(cpts,cpts);
z=sr*sin(pp)+center_p(3)*ones(cpts,cpts);
cdata=ones(cpts,cpts,3);
cdata(:,:,1)=202/255*ones(cpts,cpts,1);
cdata(:,:,2)=0/255*ones(cpts,cpts,1);
cdata(:,:,3)=0/255*ones(cpts,cpts,1);
surf(x,y,z,cdata,'FaceAlpha',0.2,'EdgeAlpha',0);