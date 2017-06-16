function J=calculate_sys_J(joint_v)

q1=joint_v(1); q2=joint_v(2); q3=joint_v(3); q4=joint_v(4); q5=joint_v(5); q6=joint_v(6);theta=joint_v(7);delta=joint_v(8);
d1=475; a1=180; d2=150+96; d3=d2;a2=385; a3=100; d4=445;d6=90;L=40;

x_axis=[1;0;0]; y_axis=[0;1;0]; z_axis=[0;0;1];
%kinematics

R_0_1=rotz(pi/2+q1);
% p_0_o1=[0;0;d1];

R_1_2=rotx(-pi/2)*rotz(-pi/2+q2);
R_0_2=R_0_1*R_1_2;
% p_0_o2=p_0_o1+a1*R_0_1*x_axis+d2*R_0_1*y_axis;

R_2_3=rotz(q3);
R_0_3=R_0_2*R_2_3;
% p_0_o3=p_0_o2+a2*R_0_2*x_axis-d3*R_0_2*z_axis;

R_3_4=rotx(-pi/2)*rotz(q4);
R_0_4=R_0_3*R_3_4;
% p_0_o4=p_0_o3+a3*R_0_3*x_axis+d4*R_0_4*z_axis;


R_4_5=rotx(pi/2)*rotz(q5);
R_0_5=R_0_4*R_4_5;


R_5_6=rotx(-pi/2)*rotz(q6);
R_0_6=R_0_5*R_5_6;


R_6_7=rotz(-delta)*roty(theta)*rotz(delta);

toc;
t_22=toc

sin_theta=sin(theta);
cos_theta=cos(theta);
sin_delta=sin(delta);
cos_delta=cos(delta);


if theta>0
Jcv=R_0_6*([L*sin_delta*(theta*sin_theta+cos_theta-1)/theta^2,L*sin_delta*(cos_theta-1)/theta;...
    -L*sin_delta*(theta*sin_theta+cos_theta-1)/theta^2,L*cos_delta*(cos_theta-1)/theta;...
    L*(theta*cos_theta-sin_theta)/theta^2,0]...
    +15*[cos_delta*cos_theta,-sin_delta*sin_theta;...
    -sin_delta*cos_theta,-cos_delta*sin_theta;...
    -sin_theta,0]);
Jcw=R_0_6*[sin_delta,cos_delta*sin_theta;cos_delta,-sin_delta*sin_theta;0,cos_theta-1];

p_6_o6g=580*z_axis+L/theta*[cos_delta*(1-cos_theta);sin_delta*(cos_theta-1);sin_theta]+R_6_7*[0;0;15];
else 
    Jcv=R_0_6*[(L+15)/2*cos_delta,0;-(L+15)/2*sin_delta,0;0,0];
    Jcw=R_0_6*[sin_delta,0;cos_delta,0;0,0];
    p_6_o6g=580*z_axis+(L+15)*[0;0;1];
end


Jc=[Jcv;Jcw];
p_5_o5o6=d6*R_5_6*z_axis;
p_4_o4o5=[0;0;0];
p_3_o3o4=a3*x_axis+d4*R_3_4*z_axis;
p_2_o2o3=a2*x_axis-d3*R_2_3*z_axis;
p_1_o1o2=a1*x_axis+d2*R_1_2*z_axis;

cross_z=[0,-1,0;1,0,0;0,0,0];

% J6v=R_0_6*(cross(z_axis,p_6_o6g));
J6v=R_0_6*(cross_z*p_6_o6g);
J6w=R_0_6*z_axis;
J6=[J6v;J6w];

p_5_o5g=p_5_o5o6+R_5_6*p_6_o6g;
% J5v=R_0_5*(cross(z_axis,p_5_o5g));
J5v=R_0_5*(cross_z*p_5_o5g);
J5w=R_0_5*z_axis;
J5=[J5v;J5w];

p_4_o4g=p_4_o4o5+R_4_5*p_5_o5g;
% J4v=R_0_4*(cross(z_axis,p_4_o4g));
J4v=R_0_4*(cross_z*p_4_o4g);
J4w=R_0_4*z_axis;
J4=[J4v;J4w];

p_3_o3g=p_3_o3o4+R_3_4*p_4_o4g;
% J3v=R_0_3*(cross(z_axis,p_3_o3g));
J3v=R_0_3*(cross_z*p_3_o3g);
J3w=R_0_3*z_axis;
J3=[J3v;J3w];

p_2_o2g=p_2_o2o3+R_2_3*p_3_o3g;
% J2v=R_0_2*(cross(z_axis,p_2_o2g));
J2v=R_0_2*(cross_z*p_2_o2g);
J2w=R_0_2*z_axis;
J2=[J2v;J2w];

p_1_o1g=p_1_o1o2+R_1_2*p_2_o2g;
% J1v=R_0_1*(cross(z_axis,p_1_o1g));
J1v=R_0_1*(cross_z*p_1_o1g);
J1w=R_0_1*z_axis;
J1=[J1v;J1w];

% J=[Jc,J6,J5,J4,J3,J2,J1];
J=[J1,J2,J3,J4,J5,J6,Jc];

toc;
t_11=toc

end