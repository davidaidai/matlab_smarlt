%% generate Cd
npts=1000;
z_axis=[0;0;1];
s_tube_fir=200; 
s_tube_sec=200;
tube_theta_fir=-30/180*pi;
tube_theta_sec=75/180*pi;
tube_r_fir=s_tube_fir/tube_theta_fir; 
tube_r_sec=s_tube_sec/tube_theta_sec;
R_0_6=eye(3,3);
R_6_t2=rotx(-tube_theta_fir)*rotx(-tube_theta_sec); 
R_0_t2=R_0_6*R_6_t2;
draw_t_theta_fir=linspace(0,tube_theta_fir,npts/2);
draw_t_theta_sec=linspace(0,tube_theta_sec,npts/2);
if tube_theta_fir==0
    p_6_t2_fir=180*z_axis*ones(1,npts/2)+[zeros(1,npts/2);zeros(1,npts/2);linspace(0,s_tube_fir,npts/2)];
    p_6_t2_sec=p_6_t2_fir(:,npts/2)*ones(1,npts/2)+rotx(-tube_theta_fir)*[zeros(1,npts/2);tube_r_sec-cos(draw_t_theta_sec)*tube_r_sec;sin(draw_t_theta_sec)*tube_r_sec];
elseif tube_theta_sec==0
    p_6_t2_fir=180*z_axis*ones(1,npts/2)+[zeros(1,npts/2);tube_r_fir-cos(draw_t_theta_fir)*tube_r_fir;sin(draw_t_theta_fir)*tube_r_fir];
    p_6_t2_sec=p_6_t2_fir(:,npts/2)*ones(1,npts/2)+rotx(-tube_theta_fir)*[zeros(1,npts/2);zeros(1,npts/2);linspace(0,s_tube_sec,npts/2)];
else
p_6_t2_fir=180*z_axis*ones(1,npts/2)+[zeros(1,npts/2);tube_r_fir-cos(draw_t_theta_fir)*tube_r_fir;sin(draw_t_theta_fir)*tube_r_fir];
p_6_t2_sec=p_6_t2_fir(:,npts/2)*ones(1,npts/2)+rotx(-tube_theta_fir)*[zeros(1,npts/2);tube_r_sec-cos(draw_t_theta_sec)*tube_r_sec;sin(draw_t_theta_sec)*tube_r_sec];
end
p_6_t2=[p_6_t2_fir,p_6_t2_sec];
Cd=p_6_t2;

%% 
i=1;m=1;eps=0.1;
while i~=size(Cd,2)
v=findV(i,Cd,eps);
p(:,m)=Cd(:,v);
m=m+1;
i=v;
end

