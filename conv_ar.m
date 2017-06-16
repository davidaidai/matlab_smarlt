function [angle_axis,p_omni]=conv_ar(omni_data)
r_11=omni_data(1);
r_22=omni_data(6);
r_33=omni_data(11);
r_32=omni_data(7);
r_23=omni_data(10);
r_13=omni_data(9);
r_31=omni_data(3);
r_21=omni_data(2);
r_12=omni_data(5);
p_omni=omni_data(13:15);
rdiag=r_11+r_22+r_33;
if rdiag-1~=0
    theta=acos((rdiag-1)/2);
    r_x=(r_32-r_23)*0.5/sin(theta);
    r_y=(r_13-r_31)*0.5/sin(theta);
    r_z=(r_21-r_12)*0.5/sin(theta);
else if r_23+r_32+r_13+r_31==0
        r_x=0;r_y=0;r_z=1;
    else
        r_x=sqrt(0.5*(r_11+1));
        r_y=r_12/(2*r_x);
        r_z=r_13/(2*r_x);
    end
end
angle_axis=[theta,r_x,r_y,r_z,p_omni];

end