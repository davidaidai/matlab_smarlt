function [p_c,R_c,p_troc,R_troc,J_grip,J_denso,J_rcm_p]=denso_kinematic(joint_v,s)

q1=joint_v(1); q2=joint_v(2); q3=joint_v(3); q4=joint_v(4); q5=joint_v(5); q6=joint_v(6);theta=joint_v(7);delta=joint_v(8);
d1=475; a1=180; d2=150+96; d3=d2;a2=385; a3=100; d4=445;d6=90;L=40;
x_axis=[1;0;0]; y_axis=[0;1;0]; z_axis=[0;0;1];
%%%%%%%%%%
R_0_1=rotz(pi/2+q1);
p_0_o1=[0;0;d1];

R_1_2=rotx(-pi/2)*rotz(-pi/2+q2);
R_0_2=R_0_1*R_1_2;
p_0_o2=p_0_o1+a1*R_0_1*x_axis+d2*R_0_1*y_axis;

R_2_3=rotz(-pi/2+q3);
R_0_3=R_0_2*R_2_3;
p_0_o3=p_0_o2+a2*R_0_2*x_axis-d3*R_0_2*z_axis;

R_3_4=rotx(-pi/2)*rotz(q4);
R_0_4=R_0_3*R_3_4;
p_0_o4=p_0_o3+a3*R_0_3*x_axis+d4*R_0_4*z_axis;

R_4_5=rotx(pi/2)*rotz(q5);
R_0_5=R_0_4*R_4_5;
p_0_o5=p_0_o4;

R_5_6=rotx(-pi/2)*rotz(q6);
R_0_6=R_0_5*R_5_6;
p_0_o6=p_0_o5+d6*R_0_6*z_axis;

R_6_7=rotz(-delta)*roty(theta)*rotz(delta);
R_0_7=R_0_6*R_6_7;

sin_theta=sin(theta);
cos_theta=cos(theta);
sin_delta=sin(delta);
cos_delta=cos(delta);

if theta>0
    p_6_c=L/theta*[cos_delta*(1-cos_theta);sin_delta*(cos_theta-1);sin_theta];
    p_0_o7=p_0_o6+580*R_0_6*z_axis+R_0_6*p_6_c;
else
    p_0_o7=p_0_o6+580*R_0_6*z_axis+R_0_6*[0;0;L];
end
p_0_g=p_0_o7+15*R_0_7*z_axis;
        
R_c=R_0_7;
p_c=p_0_g;   

R_troc=R_0_6;
p_troc=p_0_o6+s*R_0_6*z_axis;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Jacobian
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

%%%%%%%%%%%%%%%
% J_gripper
%%%%%%%%%%%%%%
J_grip=[J1,J2,J3,J4,J5,J6,Jc];
%%%%%%%%%%%
% J_denso
%%%%%%%%%%%%
p_6_o6=[0;0;0];
J6vd=R_0_6*(cross_z*p_6_o6);
J6wd=R_0_6*z_axis;
J6d=[J6vd;J6wd];

p_5_o5o6=p_5_o5o6+R_5_6*p_6_o6;
J5vd=R_0_5*(cross_z*p_5_o5o6);
J5wd=R_0_5*z_axis;
J5d=[J5vd;J5wd];

p_4_o4o6=p_4_o4o5+R_4_5*p_5_o5o6;
J4vd=R_0_4*(cross_z*p_4_o4o6);
J4wd=R_0_4*z_axis;
J4d=[J4vd;J4wd];

p_3_o3o6=p_3_o3o4+R_3_4*p_4_o4o6;
J3vd=R_0_3*(cross_z*p_3_o3o6);
J3wd=R_0_3*z_axis;
J3d=[J3vd;J3wd];

p_2_o2o6=p_2_o2o3+R_2_3*p_3_o3o6;
J2vd=R_0_2*(cross_z*p_2_o2o6);
J2wd=R_0_2*z_axis;
J2d=[J2vd;J2wd];

p_1_o1o6=p_1_o1o2+R_1_2*p_2_o2o6;
J1vd=R_0_1*(cross_z*p_1_o1o6);
J1wd=R_0_1*z_axis;
J1d=[J1vd;J1wd];

J_denso=[J1d,J2d,J3d,J4d,J5d,J6d];

%%%%%%%%%%%%%%%%
% J_rcm_p
%%%%%%%%%%%%%%%
J7vd=[0;0;0];
J8vd=[0;0;0];
J7wd=[0;0;0];
J8wd=[0;0;0];
J_vd=[J1vd,J2vd,J3vd,J4vd,J5vd,J6vd,J7vd,J8vd];
J_wd=[J1wd,J2wd,J3wd,J4wd,J5wd,J6wd,J7wd,J8wd];

z_0_6=R_0_6(:,3);
p_0_o6rcm=R_0_6*[0;0;s];
p_0_rcmc=cross_matric(p_0_o6rcm);

 J_rcm_p=(eye(3,3)-z_0_6*transpose(z_0_6))*(J_vd-p_0_rcmc*J_wd);

end
