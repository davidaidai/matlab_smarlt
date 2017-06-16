function [p_c,R_c]=cal_pose(joint_v)
q1=joint_v(1); q2=joint_v(2); q3=joint_v(3); q4=joint_v(4); q5=joint_v(5); q6=joint_v(6);theta=joint_v(7);delta=joint_v(8);
d1=475; a1=180; d2=150+96; d3=d2;a2=385; a3=100; d4=445;d6=90;

x_axis=[1;0;0]; y_axis=[0;1;0]; z_axis=[0;0;1];
%kinematics

R_0_1=rotz(pi/2+q1);
p_0_o1=[0;0;d1];

R_1_2=rotx(-pi/2)*rotz(-pi/2+q2);
R_0_2=R_0_1*R_1_2;
p_0_o2=p_0_o1+a1*R_0_1*x_axis+d2*R_0_1*y_axis;

R_2_3=rotz(q3);
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

if theta<=0
    delta=delta+pi;
    theta=-theta;
end

R_6_7=rotz(-delta)*roty(theta)*rotz(delta);
R_0_7=R_0_6*R_6_7;
if theta>0
    p_6_c=40/theta*[cos(delta)*(1-cos(theta));sin(delta)*(cos(theta)-1);sin(theta)];
    p_0_o7=p_0_o6+580*R_0_6*z_axis+R_0_6*p_6_c;
else
    p_0_o7=p_0_o6+580*R_0_6*z_axis+R_0_6*[0;0;40];
end
p_0_g=p_0_o7+15*R_0_7*z_axis;
    
    
R_c=R_0_7;
p_c=p_0_g;    
    
 
end

    



