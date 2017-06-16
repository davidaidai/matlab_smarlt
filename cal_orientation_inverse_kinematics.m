syms theta4 theta5 theta6
R_3_4=[1,0,0;0,0,1;0,-1,0]*[cos(theta4),-sin(theta4),0;sin(theta4),cos(theta4),0;0,0,1];
R_4_5=[1,0,0;0,0,-1;0,1,0]*[cos(theta5),-sin(theta5),0;sin(theta5),cos(theta5),0;0,0,1];
R_5_6=[1,0,0;0,0,1;0,-1,0]*[cos(theta6),-sin(theta6),0;sin(theta6),cos(theta6),0;0,0,1];
R_3_6=R_3_4*R_4_5*R_5_6