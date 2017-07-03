function [mov_p_o0,q_c]=initialize_denso_configuration_with_rcm_target(p_0_wristcenter)
q_c=[(-170/180*pi+170/180*pi)/2;(-90/180*pi+135/180*pi)/2;(-80/180*pi+168/180*pi)/2;0*pi/180;90*pi/180;0*pi/180;45/180*pi;-90/180*pi];
% q_c=[(-170/180*pi+170/180*pi)/2;(-90/180*pi+135/180*pi)/2;(-80/180*pi+168/180*pi)/2;0*pi/180;90*pi/180;0*pi/180;0/180*pi;0/180*pi];
q1=q_c(1); q2=q_c(2); q3=q_c(3);q4=q_c(4);

R_0_1=rotz(pi/4)*rotz(q1);

R_1_2=rotx(-pi/2)*rotz(-pi/2+q2);
R_0_2=R_0_1*R_1_2;

R_2_3=rotz(-pi/2+q3);
R_0_3=R_0_2*R_2_3;

%DENSO-VM6083
%d1=475; a1=180; d2=150+96; d3=d2;a2=385; a3=100; d4=445;d6=90;L=40;
%DENSO-VM60B1?
d1=475; a1=180; d2=150+96; d3=d2;a2=520; a3=100; d4=590;d6=90;

x_axis=[1;0;0]; y_axis=[0;1;0]; z_axis=[0;0;1];

%%%%%%%%%%
p_0_o1=[0;0;d1];
p_0_o2=p_0_o1+a1*R_0_1*x_axis+d2*R_0_1*y_axis;
p_0_o3=p_0_o2+a2*R_0_2*x_axis-d3*R_0_2*z_axis;
R_3_4=rotx(-pi/2)*rotz(q4);
R_0_4=R_0_3*R_3_4;
p_0_o4=p_0_o3+a3*R_0_3*x_axis+d4*R_0_4*z_axis;
mov_p_o0=p_0_wristcenter-p_0_o4;

end
