function joints = convert2motorJoints(q_c,griper_c)
%将当前关节值转换为当前电机或者Denso机械臂的位置量
% Denso Joints 
joints(1:6)=q_c(1:6)*180/pi;
% theta and delta joints
%distal distance 
q_x=-(q_c(7))*2.5*cos(q_c(8));
q_y=(q_c(7))*2.5*sin(q_c(8));

blockx=q_x/48*60;
blocky=q_y/48*60;

motor1=blockx/2/pi*37*1024;
motor2=blocky/2/pi*37*1024;
% q_x=q_x/1.25;
% q_y=q_y/1.25;
% 
% % calculate quadcounts
% 
% q_x=q_x/pi/6*18/40*370*1024;
% q_y=q_y/pi/6*18/40*370*1024;
joints(7)=motor1;
joints(8)=motor2;
joints(9)=griper_c;

end

