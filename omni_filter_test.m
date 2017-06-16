%%%%%%%%%%omni test
m=1;
n=1;
k=1;
unfilt_ang_axis=zeros(1000,7);

while m<=1000
[omni_data,button]=getomni();
angle_axis=conv_ar(omni_data);
unfilt_ang_axis(m,:)=angle_axis;
m=m+1;
end

filt5_ang_axis=zeros(990,7);
while n<=990
angle_axis_med=median(unfilt_ang_axis(n:n+4,:));
r_x=angle_axis_med(2);
r_y=angle_axis_med(3);
r_z=angle_axis_med(4);
r_l=sqrt(r_x*r_x+r_y*r_y+r_z*r_z);
angle_axis_med(2)=r_x/r_l;
angle_axis_med(3)=r_y/r_l;
angle_axis_med(4)=r_z/r_l;
filt5_ang_axis(n,:)=angle_axis_med;
n=n+1;    
end

filt50_ang_axis=zeros(950,7);
while k<=950
angle_axis_med=median(unfilt_ang_axis(k:k+49,:));
r_x=angle_axis_med(2);
r_y=angle_axis_med(3);
r_z=angle_axis_med(4);
r_l=sqrt(r_x*r_x+r_y*r_y+r_z*r_z);
angle_axis_med(2)=r_x/r_l;
angle_axis_med(3)=r_y/r_l;
angle_axis_med(4)=r_z/r_l;
filt50_ang_axis(k,:)=angle_axis_med;
k=k+1;    
end




