function [troc_para]=cal_moved_troc(joint_v,troc_para,p_0_t1former)
% q1=joint_v(1)*180/pi; q2=joint_v(2)*180/pi; q3=joint_v(3)*180/pi; q4=joint_v(4)*180/pi; q5=joint_v(5)*180/pi; q6=joint_v(6)*180/pi;
q1=joint_v(1); q2=joint_v(2); q3=joint_v(3); q4=joint_v(4); q5=joint_v(5); q6=joint_v(6);

t_theta1=troc_para(1); % RCM圆弧圆心角
t_theta2=troc_para(2); % trocar末端点圆弧圆心角
t_r=troc_para(3); % 圆弧半径

d1=475; a1=180; d2=150+96; d3=d2;a2=385; a3=100; d4=445;d6=90;

x_axis=[1;0;0]; y_axis=[0;1;0]; z_axis=[0;0;1];
%kinematics

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

R_6_t1=rotx(-t_theta1);
p_6_o6t1=180*z_axis+[0;t_r-cos(t_theta1)*t_r;sin(t_theta1)*t_r];

R_0_t1=R_0_6*R_6_t1;
p_0_t1=p_0_o6+R_0_6*p_6_o6t1;    

%the trocar point after motion
p_0_t1mov=p_0_t1-p_0_t1former;
if norm(p_0_t1mov)==0;
    t_theta1_after=t_theta1;
else
    %          dets=norm(p_mov)*(p_mov'*R_former(:,3)/norm(p_mov'*R_former(:,3)));
    %          dets=norm(p_mov)*(p_mov'*R_after(:,3)/norm(p_mov'*R_after(:,3)));% 大小由norm(p_mov)决定，方向由p_mov向量与R_0_6第三列z向量的夹角确定
    p_6_rcmafter=R_0_6'*(p_0_t1former-p_0_o6)-180*z_axis;
    t_theta1_after=asin(p_6_rcmafter(3)/t_r);
end
t_theta1=t_theta1_after;
troc_para=[t_theta1,t_theta2,t_r];
 
end
