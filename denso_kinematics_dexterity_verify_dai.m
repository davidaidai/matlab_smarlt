function [p_c,R_c,p_0_rcm,R_0_rcm,J_grip,J_denso,J_rcm_p]=denso_kinematics_dexterity_verify_dai(mov_p_o0,joint_v,tube_para)

q1=joint_v(1); q2=joint_v(2); q3=joint_v(3); q4=joint_v(4); q5=joint_v(5); q6=joint_v(6);theta=joint_v(7);delta=joint_v(8);
%DENSO-VM6083
%d1=475; a1=180; d2=150+96; d3=d2;a2=385; a3=100; d4=445;d6=90;L=40;
%DENSO-VM60B1?
d1=475; a1=180; d2=150+96; d3=d2;a2=520; a3=100; d4=590;d6=90;L=40;

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

%%%%%%%%%%%%%%%%%%%%%%%%%
% RCM point
s_rcm=tube_para(1); % RCM点弧长
s_tube_fir=tube_para(2); % 第一段弧长
s_tube_sec=tube_para(3); % 第二段弧长
tube_theta_fir=tube_para(4); % 第一段圆弧圆心角
tube_theta_sec=tube_para(5); % 第二段圆弧圆心角
tube_r_fir=s_tube_fir/tube_theta_fir;
tube_r_sec=s_tube_sec/tube_theta_sec;
if s_rcm>=s_tube_fir+s_tube_sec
    s_rcm=s_tube_fir+s_tube_sec;
elseif s_rcm<=0
    s_rcm=0;
end
if s_rcm<s_tube_fir && s_rcm>=0
    R_6_t1=rotx(-s_rcm/tube_r_fir);
    if tube_theta_fir==0
        p_6_o6t1=180*z_axis+[0;0;s_rcm];
    else
        p_6_o6t1=180*z_axis+[0;tube_r_fir-cos(s_rcm/tube_r_fir)*tube_r_fir;sin(s_rcm/tube_r_fir)*tube_r_fir];
    end
    R_0_rcm=R_0_6*R_6_t1;
    p_0_rcm=p_0_o6+R_0_6*p_6_o6t1;
elseif s_rcm>=s_tube_fir && s_rcm<=s_tube_fir+s_tube_sec
    R_6_t1=rotx(-tube_theta_fir)*rotx(-(s_rcm-s_tube_fir)/tube_r_sec);
    if tube_theta_fir==0
        p_6_o6t1=180*z_axis+[0;0;s_tube_fir]+rotx(-tube_theta_fir)*[0;tube_r_sec-cos((s_rcm-s_tube_fir)/tube_r_sec)*tube_r_sec;sin((s_rcm-s_tube_fir)/tube_r_sec)*tube_r_sec];
    elseif tube_theta_sec==0
        p_6_o6t1=180*z_axis+[0;tube_r_fir-cos(tube_theta_fir)*tube_r_fir;sin(tube_theta_fir)*tube_r_fir]+rotx(-tube_theta_fir)*[0;0;s_rcm-s_tube_fir];
    else
        p_6_o6t1=180*z_axis+[0;tube_r_fir-cos(tube_theta_fir)*tube_r_fir;sin(tube_theta_fir)*tube_r_fir]+rotx(-tube_theta_fir)*[0;tube_r_sec-cos((s_rcm-s_tube_fir)/tube_r_sec)*tube_r_sec;sin((s_rcm-s_tube_fir)/tube_r_sec)*tube_r_sec];
    end
    R_0_rcm=R_0_6*R_6_t1;
    p_0_rcm=p_0_o6+R_0_6*p_6_o6t1;
end
R_6_t2=rotx(-tube_theta_fir)*rotx(-tube_theta_sec); % troc末端在6坐标系中的指向
R_0_t2=R_0_6*R_6_t2;
if tube_theta_fir==0
    p_6_o6t2=180*z_axis+[0;0;s_tube_fir]+rotx(-tube_theta_fir)*[0;s_tube_sec/tube_theta_sec-cos(tube_theta_sec)*s_tube_sec/tube_theta_sec;sin(tube_theta_sec)*s_tube_sec/tube_theta_sec];
elseif tube_theta_sec==0
    p_6_o6t2=180*z_axis+[0;s_tube_fir/tube_theta_fir-cos(tube_theta_fir)*s_tube_fir/tube_theta_fir;sin(tube_theta_fir)*s_tube_fir/tube_theta_fir]+rotx(-tube_theta_fir)*[0;0;s_tube_sec];
else
    p_6_o6t2=180*z_axis+[0;s_tube_fir/tube_theta_fir-cos(tube_theta_fir)*s_tube_fir/tube_theta_fir;sin(tube_theta_fir)*s_tube_fir/tube_theta_fir]+rotx(-tube_theta_fir)*[0;s_tube_sec/tube_theta_sec-cos(tube_theta_sec)*s_tube_sec/tube_theta_sec;sin(tube_theta_sec)*s_tube_sec/tube_theta_sec];
end
p_0_o6t2=R_0_6*p_6_o6t2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%



R_t2_c=rotz(-delta)*roty(theta)*rotz(delta);
R_6_c=R_6_t2*R_t2_c;
R_0_c=R_0_t2*R_t2_c;

sin_theta=sin(theta);
cos_theta=cos(theta);
sin_delta=sin(delta);
cos_delta=cos(delta);

if theta>0
    p_t2_c=L/theta*[cos_delta*(1-cos_theta);sin_delta*(cos_theta-1);sin_theta]; %tool根部指向末端的向量
    p_0_c=p_0_o6+p_0_o6t2+R_0_t2*p_t2_c;
else
    p_0_c=p_0_o6+p_0_o6t2+R_0_t2*[0;0;L]; %theta永远不会小于0
end
p_0_g=p_0_c+15*R_0_c*z_axis;

R_c=R_0_c;
p_c=p_0_g;




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Jacobian
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if theta>0
    Jcv=R_0_t2*([L*cos_delta*(theta*sin_theta+cos_theta-1)/theta^2,L*sin_delta*(cos_theta-1)/theta;...
        -L*sin_delta*(theta*sin_theta+cos_theta-1)/theta^2,L*cos_delta*(cos_theta-1)/theta;...
        L*(theta*cos_theta-sin_theta)/theta^2,0]...
        +15*[cos_delta*cos_theta,-sin_delta*sin_theta;...
        -sin_delta*cos_theta,-cos_delta*sin_theta;...
        -sin_theta,0]);
    Jcw=R_0_t2*[sin_delta,cos_delta*sin_theta;cos_delta,-sin_delta*sin_theta;0,cos_theta-1];
    
    p_6_o6g=p_6_o6t2+L/theta*[cos_delta*(1-cos_theta);sin_delta*(cos_theta-1);sin_theta]+R_6_c*[0;0;15];
else
    Jcv=R_0_t2*[L/2*cos_delta+15*cos_delta,0;-L/2*sin_delta-15*sin_delta,0;0,0];
    Jcw=R_0_t2*[sin_delta,0;cos_delta,0;0,0];
    p_6_o6g=p_6_o6t2+(L+15)*z_axis;
end


Jc=[Jcv;Jcw];
p_5_o5o6=d6*R_5_6*z_axis;
p_4_o4o5=[0;0;0];
p_3_o3o4=a3*x_axis+d4*R_3_4*z_axis;
p_2_o2o3=a2*x_axis-d3*R_2_3*z_axis;
p_1_o1o2=a1*x_axis+d2*R_1_2*z_axis;

cross_z=[0,-1,0;1,0,0;0,0,0]; % z_axis的反对称矩阵

J6v=R_0_6*(cross_z*p_6_o6g);
J6w=R_0_6*z_axis;
J6=[J6v;J6w];

p_5_o5g=p_5_o5o6+R_5_6*p_6_o6g;
J5v=R_0_5*(cross_z*p_5_o5g);
J5w=R_0_5*z_axis;
J5=[J5v;J5w];

p_4_o4g=p_4_o4o5+R_4_5*p_5_o5g;
J4v=R_0_4*(cross_z*p_4_o4g);
J4w=R_0_4*z_axis;
J4=[J4v;J4w];

p_3_o3g=p_3_o3o4+R_3_4*p_4_o4g;
J3v=R_0_3*(cross_z*p_3_o3g);
J3w=R_0_3*z_axis;
J3=[J3v;J3w];

p_2_o2g=p_2_o2o3+R_2_3*p_3_o3g;
J2v=R_0_2*(cross_z*p_2_o2g);
J2w=R_0_2*z_axis;
J2=[J2v;J2w];

p_1_o1g=p_1_o1o2+R_1_2*p_2_o2g;
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

p_6_rcmtan=R_6_t1(:,3); % RCM point处的单位切向量
p_0_rcmtan=R_0_6*p_6_rcmtan;
p_0_o6rcm=R_0_6*p_6_o6t1;
p_0_rcmc=cross_matric(p_0_o6rcm);

J_rcm_p=(eye(3,3)-p_0_rcmtan*transpose(p_0_rcmtan))*(J_vd-p_0_rcmc*J_wd);% 3x8矩阵

end
