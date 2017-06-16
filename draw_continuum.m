function [cylinder_handles linehandles gripperhandle coo_handle]=draw_continuum(figure_handle,origin_p,origin_r,draw_flag,joint_variables,cylinder_handles,linehandles,gripperhandle,coo_handle)
%%%set global variables\color\transparency variables
global r_disk;global h_disk;global r_2nd_bb;global number;
r_disk=3;h_disk=0.6;r_2nd_bb=2.5;number=20; Lg=15;
color_blue=[0;0;1];color_green=[0;1;0];color_red=[1;0;0];color_yellow=[1;1;0];color_light=[0.9;0.9;0.9];color_light2=[0.0;0.5;0.5];
transprancy=[0.5;0.1;0.8];
%length of wire
length_of_wire=40;
%draw the second_segment
orientation=origin_r;
origin=origin_p;
%%joint varible3 and 4
delta=joint_variables(1);
if joint_variables(2)>=0
   theta=joint_variables(2);%from 0бу-90бу
else
    delta=delta+pi;
    theta=-joint_variables(2);
end
Rb1_c1=orientation*rotd([0;0;1],-delta);
%%%draw the 1st backbone of segment
center_1st_bb=origin;
center_2nd_bb_1=center_1st_bb+orientation*rotd([0;0;1],0)*[r_2nd_bb;0;0];
center_2nd_bb_2=center_1st_bb+orientation*rotd([0;0;1],pi*2/3)*[r_2nd_bb;0;0];
center_2nd_bb_3=center_1st_bb+orientation*rotd([0;0;1],pi*4/3)*[r_2nd_bb;0;0];
Rb1_e2=Rb1_c1*rotd([0;1;0],theta);
%define linehandle for each line of 2nd segment
linehandle1=linehandles(1,1:1*number);
linehandle2=linehandles(1,1*number+1:2*number);
linehandle3=linehandles(1,2*number+1:3*number);
linehandle4=linehandles(1,3*number+1:4*number);
if theta>0   
   R_of_wire=length_of_wire/(theta);
   origin2=center_1st_bb+Rb1_c1*[R_of_wire*(1-cos(theta));0;R_of_wire*sin(theta)];
else
    R_of_wire=length_of_wire;
    origin2=center_1st_bb+Rb1_c1*[0;0;length_of_wire];
end

%draw the base disk 
center_1=origin;
center_2=origin+h_disk*orientation(:,3);
axis_x_1=origin+r_disk*orientation(:,1);
cylinder_handle=cylinder_handles(1,:);
cylinder_handle=draw_cylinderb(figure_handle,center_1,axis_x_1,center_2,color_light2,transprancy(3),draw_flag,cylinder_handle);hold on;  

%%%draw the backbones of segment2
  [spacer_points2 linehandle1]=draw_1st_bb(figure_handle,center_1st_bb,origin,theta,Rb1_c1,R_of_wire,draw_flag,2,linehandle1,color_yellow,2);
  linehandle2=draw_2nd_bb(figure_handle,center_2nd_bb_1,center_2nd_bb_1,theta,Rb1_c1,R_of_wire,draw_flag,-delta,0,linehandle2,color_red,2);
  linehandle3=draw_2nd_bb(figure_handle,center_2nd_bb_2,center_2nd_bb_2,theta,Rb1_c1,R_of_wire,draw_flag,-delta,1,linehandle3,color_green,2);
  linehandle4=draw_2nd_bb(figure_handle,center_2nd_bb_3,center_2nd_bb_3,theta,Rb1_c1,R_of_wire,draw_flag,-delta,2,linehandle4,color_blue,2);
%draw the spacer disks for 2nd segment
space_cylinder_handle1=zeros(2,18);
for i=1:2
space_center1=spacer_points2(:,i);
space_rotation_matrix=Rb1_c1*rotd([0;1;0],i*theta/3);
space_center2=space_center1+space_rotation_matrix*[0;0;-h_disk];
space_axis=space_center1+space_rotation_matrix*[r_disk;0;0];
space_cylinder_handle1(i,:)=cylinder_handles(i+1,:);
space_cylinder_handle1(i,:)=draw_cylinderb(figure_handle,space_center1,space_axis,space_center2,color_light2,transprancy(1),draw_flag,space_cylinder_handle1(i,:));hold on;
end

%draw the end disk 
center_3=origin2;
center_4=origin2+Rb1_e2*[0;0;-h_disk];
axis_x_2=origin2+Rb1_e2*[r_disk;0;0];
cylinder_handle1=cylinder_handles(4,:);
cylinder_handle1=draw_cylinderb(figure_handle,center_3,axis_x_2,center_4,color_light2,transprancy(3),draw_flag,cylinder_handle1);hold on;  
%%draw a needle
Rb1_g2=Rb1_e2*rotd([0;0;1],delta);%coordinate transform matrix of tg2
sigma=0;% the joint variable for the wrist
Rb1_c3=Rb1_g2*rotd([0;0;1],sigma);
Length_gripper=15;
center_8=center_3+Rb1_c3*[0;0;Length_gripper];
axis_x_5=center_4+Rb1_c3*[r_disk;0;0];
gripperhandle=draw_gripper(figure_handle,center_4,axis_x_5,center_8,color_red,transprancy(3),draw_flag,gripperhandle);hold on;
cylinder_handles=[cylinder_handle;space_cylinder_handle1;cylinder_handle1];
linehandles=[linehandle1 linehandle2 linehandle3 linehandle4];
%draw the coordinate
coo_handle=draw_coordinate_system2(figure_handle,30,Rb1_g2,center_8,'rgb',draw_flag,coo_handle);hold on;
end

