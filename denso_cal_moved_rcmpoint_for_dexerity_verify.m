function [tube_para]=denso_cal_moved_rcmpoint_for_dexerity_verify(mov_p_o0,joint_v,tube_para,p_rcm_origin)
% q1=joint_v(1)*180/pi; q2=joint_v(2)*180/pi; q3=joint_v(3)*180/pi; q4=joint_v(4)*180/pi; q5=joint_v(5)*180/pi; q6=joint_v(6)*180/pi;
q1=joint_v(1); q2=joint_v(2); q3=joint_v(3); q4=joint_v(4); q5=joint_v(5); q6=joint_v(6);

%DENSO-VM6083
%d1=475; a1=180; d2=150+96; d3=d2;a2=385; a3=100; d4=445;d6=90;L=40;
%DENSO-VM60B1?
d1=475; a1=180; d2=150+96; d3=d2;a2=520; a3=100; d4=590;d6=90;L=40;

x_axis=[1;0;0]; y_axis=[0;1;0]; z_axis=[0;0;1];
%kinematics

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
s_tube_fir=tube_para(2);
s_tube_sec=tube_para(3);
tube_theta_fir=tube_para(4);
tube_theta_sec=tube_para(5);
tube_r_fir=s_tube_fir/tube_theta_fir; % 第一段圆弧半径
tube_r_sec=s_tube_sec/tube_theta_sec; % 第二段圆弧半径
if tube_theta_fir==0
    p_6_startpoint_fir=[0;0;0];
    p_6_endpoint_fir=[0;0;s_tube_fir];
else
    p_6_startpoint_fir=[0;0;0];
    p_6_endpoint_fir=[0;tube_r_fir-cos(tube_theta_fir)*tube_r_fir;sin(tube_theta_fir)*tube_r_fir];
end
if tube_theta_sec==0
    p_fir_startpoint_sec=[0;0;0];
    p_fir_endpoint_sec=[0;0;s_tube_sec];
else
    p_fir_startpoint_sec=[0;0;0];
    p_fir_endpoint_sec=[0;tube_r_sec-cos(tube_theta_sec)*tube_r_sec;sin(tube_theta_sec)*tube_r_sec];
end

p_6_rcmorin=R_0_6'*(p_rcm_origin-p_0_o6)-180*z_axis; % p_0_o6+R_0_6*(180*z_axis+p_6_rcmorin)=p_rcm_oringin

if tube_theta_fir==0
    p_6_projectpoint_fir=[0;0;p_6_rcmorin(3)];
    s_rcm_fir=p_6_rcmorin(3);
    if s_rcm_fir>=0 && s_rcm_fir<=s_tube_fir
        d_pp_rcm_fir=norm(p_6_rcmorin-p_6_projectpoint_fir);
    else
        [d_pp_rcm_fir,i_fir]=min([norm(p_6_rcmorin-p_6_startpoint_fir),norm(p_6_rcmorin-p_6_endpoint_fir)]);
        if i_fir==1
            s_rcm_fir=0;
        else
            s_rcm_fir=s_tube_fir;
        end
    end
else %目前并没考虑圆弧超过90度后projection point位置的计算方法；并没考虑RCM点在垂直于圆弧所在平面且经过圆弧圆心的直线上的情况，因为RCM点不可能离圆弧那么远
    p_6_projectpoint_fir=[0;sign(p_6_rcmorin(3))*abs(tube_r_fir)/sqrt((p_6_rcmorin(2)-tube_r_fir)^2+p_6_rcmorin(3)^2)*(p_6_rcmorin(2)-tube_r_fir)+tube_r_fir;sign(p_6_rcmorin(3))*abs(tube_r_fir)/sqrt((p_6_rcmorin(2)-tube_r_fir)^2+p_6_rcmorin(3)^2)*p_6_rcmorin(3)];
    s_rcm_fir=asin(p_6_projectpoint_fir(3)/tube_r_fir)*tube_r_fir;
    if s_rcm_fir>=0 && s_rcm_fir<=s_tube_fir
        d_pp_rcm_fir=norm(p_6_rcmorin-p_6_projectpoint_fir);
    else
        [d_pp_rcm_fir,i_fir]=min([norm(p_6_rcmorin-p_6_startpoint_fir),norm(p_6_rcmorin-p_6_endpoint_fir)]);
        if i_fir==1
            s_rcm_fir=0;
        else
            s_rcm_fir=s_tube_fir;
        end
    end
end

p_fir_rcmorin=(rotx(-tube_theta_fir))'*(p_6_rcmorin-p_6_endpoint_fir); % rotx(-tube_theta_fir)*p_fir_rcmorin+p_6_endpoint_fir=p_6_rcmorin

if tube_theta_sec==0
    p_fir_projectpoint_sec=[0;0;p_fir_rcmorin(3)];
    s_rcm_sec=p_fir_rcmorin(3);
    if s_rcm_sec>=0 && s_rcm_sec<=s_tube_sec
        d_pp_rcm_sec=norm(p_fir_rcmorin-p_fir_projectpoint_sec);
    else
        [d_pp_rcm_sec,i_sec]=min([norm(p_fir_rcmorin-p_fir_startpoint_sec),norm(p_fir_rcmorin-p_fir_endpoint_sec)]);
        if i_sec==2
            s_rcm_sec=s_tube_sec;
        else
            s_rcm_sec=0;
        end
    end
else
    p_fir_projectpoint_sec=[0;sign(p_fir_rcmorin(3))*abs(tube_r_sec)/sqrt((p_fir_rcmorin(2)-tube_r_sec)^2+p_fir_rcmorin(3)^2) *(p_fir_rcmorin(2)-tube_r_sec)+tube_r_sec;sign(p_fir_rcmorin(3))*abs(tube_r_sec)/sqrt((p_fir_rcmorin(2)-tube_r_sec)^2+p_fir_rcmorin(3)^2)*p_fir_rcmorin(3)];
    s_rcm_sec=asin(p_fir_projectpoint_sec(3)/tube_r_sec)*tube_r_sec;
    if s_rcm_sec>=0 && s_rcm_sec<=s_tube_sec
        d_pp_rcm_sec=norm(p_fir_rcmorin-p_fir_projectpoint_sec);
    else
        [d_pp_rcm_sec,i_sec]=min([norm(p_fir_rcmorin-p_fir_startpoint_sec),norm(p_fir_rcmorin-p_fir_endpoint_sec)]);
        if i_sec==2
            s_rcm_sec=s_tube_sec;
        else
            s_rcm_sec=0;
        end
    end
end

if d_pp_rcm_fir<=d_pp_rcm_sec
    s_rcm_after=s_rcm_fir;
else
    s_rcm_after=s_tube_fir+s_rcm_sec;
end
    

tube_para=[s_rcm_after,s_tube_fir,s_tube_sec,tube_theta_fir,tube_theta_sec];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

