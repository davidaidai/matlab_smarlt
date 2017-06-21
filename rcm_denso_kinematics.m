function [p_0_tube,R_0_tube,J_tube]=rcm_denso_kinematics(mov_p_o0,joint_v,tube_para)

q1=joint_v(1); q2=joint_v(2); q3=joint_v(3); q4=joint_v(4); q5=joint_v(5); q6=joint_v(6);
%DENSO-VM6083
%d1=475; a1=180; d2=150+96; d3=d2;a2=385; a3=100; d4=445;d6=90;L=40;
%DENSO-VM60B1?
d1=475; a1=180; d2=150+96; d3=d2;a2=520; a3=100; d4=590;d6=90;

x_axis=[1;0;0]; y_axis=[0;1;0]; z_axis=[0;0;1];

%%%%%%%%%%
R_0_1=rotz(pi/4)*rotz(q1);
p_0_o1=[0;0;d1]+mov_p_o0;

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

% tube
s_tube_fir=tube_para(2); % tube第一段弧长
s_tube_sec=tube_para(3); % tube第一段弧长
tube_theta_fir=tube_para(4); % 第一段圆弧圆心角
tube_theta_sec=tube_para(5); % 第二段圆弧圆心角
R_6_tube=rotx(-tube_theta_fir)*rotx(-tube_theta_sec); % tube末端在6坐标系中的指向
R_0_tube=R_0_6*R_6_tube;
if tube_theta_fir==0
    p_6_o6t2=180*z_axis+[0;0;s_tube_fir]+rotx(-tube_theta_fir)*[0;s_tube_sec/tube_theta_sec-cos(tube_theta_sec)*s_tube_sec/tube_theta_sec;sin(tube_theta_sec)*s_tube_sec/tube_theta_sec];
elseif tube_theta_sec==0
    p_6_o6t2=180*z_axis+[0;s_tube_fir/tube_theta_fir-cos(tube_theta_fir)*s_tube_fir/tube_theta_fir;sin(tube_theta_fir)*s_tube_fir/tube_theta_fir]+rotx(-tube_theta_fir)*[0;0;s_tube_sec];
else
    p_6_o6t2=180*z_axis+[0;s_tube_fir/tube_theta_fir-cos(tube_theta_fir)*s_tube_fir/tube_theta_fir;sin(tube_theta_fir)*s_tube_fir/tube_theta_fir]+rotx(-tube_theta_fir)*[0;s_tube_sec/tube_theta_sec-cos(tube_theta_sec)*s_tube_sec/tube_theta_sec;sin(tube_theta_sec)*s_tube_sec/tube_theta_sec];
end
p_6_o6tube=p_6_o6t2;
p_0_tube=p_0_o6+R_0_6*p_6_o6tube;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%
% tube end Jacobian
%%%%%%%%%%%%

p_5_o5o6=d6*R_5_6*z_axis;
p_4_o4o5=[0;0;0];
p_3_o3o4=a3*x_axis+d4*R_3_4*z_axis;
p_2_o2o3=a2*x_axis-d3*R_2_3*z_axis;
p_1_o1o2=a1*x_axis+d2*R_1_2*z_axis;
cross_z=[0,-1,0;1,0,0;0,0,0]; % z_axis的反对称矩阵

J6vd=R_0_6*(cross_z*p_6_o6tube);
J6wd=R_0_6*z_axis;
J6d=[J6vd;J6wd];

p_5_o5tube=p_5_o5o6+R_5_6*p_6_o6tube;
J5vd=R_0_5*(cross_z*p_5_o5tube);
J5wd=R_0_5*z_axis;
J5d=[J5vd;J5wd];

p_4_o4tube=p_4_o4o5+R_4_5*p_5_o5tube;
J4vd=R_0_4*(cross_z*p_4_o4tube);
J4wd=R_0_4*z_axis;
J4d=[J4vd;J4wd];

p_3_o3tube=p_3_o3o4+R_3_4*p_4_o4tube;
J3vd=R_0_3*(cross_z*p_3_o3tube);
J3wd=R_0_3*z_axis;
J3d=[J3vd;J3wd];

p_2_o2tube=p_2_o2o3+R_2_3*p_3_o3tube;
J2vd=R_0_2*(cross_z*p_2_o2tube);
J2wd=R_0_2*z_axis;
J2d=[J2vd;J2wd];

p_1_o1tube=p_1_o1o2+R_1_2*p_2_o2tube;
J1vd=R_0_1*(cross_z*p_1_o1tube);
J1wd=R_0_1*z_axis;
J1d=[J1vd;J1wd];

J_tube=[J1d,J2d,J3d,J4d,J5d,J6d];



end
