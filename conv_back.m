function Omni_command=conv_back(ar_store)
angle_axis=mean(ar_store);
theta=angle_axis(1);
r_x=angle_axis(2);
r_y=angle_axis(3);
r_z=angle_axis(4);
r_l=sqrt(r_x*r_x+r_y*r_y+r_z*r_z);
r_x=r_x/r_l;
r_y=r_y/r_l;
r_z=r_z/r_l;
p_omni=angle_axis(5:7);

sin_theta=sin(theta);
cos_theta=cos(theta);
Omni_command=[r_x*r_x*(1-cos_theta)+cos_theta,r_x*r_y*(1-cos_theta)-r_z*sin_theta,r_x*r_z*(1-cos_theta)+r_y*sin_theta,p_omni(1);...
    r_x*r_y*(1-cos_theta)+r_z*sin_theta,r_y*r_y*(1-cos_theta)+cos_theta,r_y*r_z*(1-cos_theta)-r_x*sin_theta,p_omni(2);...
    r_x*r_z*(1-cos_theta)-r_y*sin_theta,r_y*r_z*(1-cos_theta)+r_x*sin_theta,r_z*r_z*(1-cos_theta)+cos_theta,p_omni(3);...
    0,0,0,1];

end