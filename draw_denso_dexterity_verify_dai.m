function [cylinderhandle, cubehandle, cylinder_handles, linehandles, gripperhandle, coo_handle,tubehandle] = draw_denso_dexterity_verify_dai(figure_handle,mov_p_o0,joint_v,tube_para,draw_flag ,denso_cylinderhandle,denso_cubehandle,denso_chandle,denso_lhandle,denso_ghandle,coo_handle,tubehandle)
%update denso according to joint variables
q1=joint_v(1); q2=joint_v(2); q3=joint_v(3); q4=joint_v(4); q5=joint_v(5); q6=joint_v(6);theta=joint_v(7);delta=joint_v(8);

%DENSO-VM6083
%d1=475; a1=180; d2=150+96; d3=d2;a2=385; a3=100; d4=445;d6=90;L=40;
%DENSO-VM60B1?
d1=475; a1=180; d2=150+96; d3=d2;a2=520; a3=100; d4=590;d6=90;L=40;

transprancy=0.5;

x_axis=[1;0;0]; y_axis=[0;1;0]; z_axis=[0;0;1];
%kinematics
R_0_0=eye(3,3);
p_0_o0=[0;0;0]+mov_p_o0;
rad_link0=150;
l_link0=290;

R_0_1=rotz(pi/4)*rotd(z_axis,q1);
p_0_o1=[0;0;d1]+p_0_o0;
rad_link1a=150;
l_link1a=d1-l_link0;
rad_link1b=107;
l_link2b=300;
p_0_o1b=p_0_o1+a1*R_0_1(:,1)+0.5*l_link2b*R_0_1(:,2);

R_1_2=rotd(x_axis,-pi/2)*rotd(z_axis,-pi/2+q2);
R_0_2=R_0_1*R_1_2;
p_0_o2=p_0_o1+a1*R_0_1*x_axis+d2*R_0_1*y_axis;
rad_link2a=107;
p_0_o2b=p_0_o2+a2*R_0_2*x_axis;
rad_link2b=84;
link2_p1=p_0_o2+rad_link2a*R_0_2*y_axis; link2_p2=p_0_o2-rad_link2a*R_0_2*y_axis; link2_p3=p_0_o2+rad_link2a*R_0_2*y_axis-(d2-150)*R_0_2*z_axis; link2_p4=p_0_o2-rad_link2a*R_0_2*y_axis-(d2-150)*R_0_2*z_axis;
link2_p5=p_0_o2b+rad_link2b*R_0_2*y_axis; link2_p6=p_0_o2b-rad_link2b*R_0_2*y_axis; link2_p7=p_0_o2b+rad_link2b*R_0_2*y_axis-(d2-150)*R_0_2*z_axis; link2_p8=p_0_o2b-rad_link2b*R_0_2*y_axis-(d2-150)*R_0_2*z_axis;
cubevertex1=[link2_p1(1),link2_p2(1),link2_p3(1),link2_p4(1),link2_p5(1),link2_p6(1),link2_p7(1),link2_p8(1);...
    link2_p1(2),link2_p2(2),link2_p3(2),link2_p4(2),link2_p5(2),link2_p6(2),link2_p7(2),link2_p8(2);...
    link2_p1(3),link2_p2(3),link2_p3(3),link2_p4(3),link2_p5(3),link2_p6(3),link2_p7(3),link2_p8(3)];

R_2_3=rotd(z_axis,-pi/2+q3);
R_0_3=R_0_2*R_2_3;
p_0_o3=p_0_o2b-d2*R_0_2*z_axis;
p_0_o3a=p_0_o3+150*R_0_3*z_axis;
rad_link3a=84;
p_0_o3b=p_0_o3+a3*R_0_3*x_axis-30*R_0_3*y_axis;
rad_link3b=58;

R_3_4=rotd(x_axis,-pi/2)*rotd(z_axis,q4);
R_0_4=R_0_3*R_3_4;
p_0_o4=p_0_o3+a3*R_0_3*x_axis+d4*R_0_4*z_axis;
rad_link4=58;
p_0_o4b=p_0_o4+58*R_0_4*y_axis;

R_4_5=rotd(x_axis,pi/2)*rotd(z_axis,q5);
R_0_5=R_0_4*R_4_5;
p_0_o5=p_0_o4;
p_0_o5a=p_0_o5+35*R_0_5*z_axis;
rad_link5=50;
rad_link5b=41;

R_5_6=rotd(x_axis,-pi/2)*rotd(z_axis,q6);
R_0_6=R_0_5*R_5_6;
p_0_o6=p_0_o5+d6*R_0_6*z_axis;
rad_link6=40;
cubevertex2=R_0_6*[-25 -25 25 25 -25 -25 25 25; 25 -25 -25 25 25 -25 -25 25; 0 0 0 0 180 180 180 180]+p_0_o6*ones(1,8);
p_0_o6b=p_0_o6+180*R_0_6*z_axis;
rad_link6b=3.5;



%%%%%%%%%%%%%%%%%%%%%%%%%
% RCM point
npts=1000;
s_tube_fir=tube_para(2); 
s_tube_sec=tube_para(3);
tube_theta_fir=tube_para(4);
tube_theta_sec=tube_para(5);
tube_r_fir=s_tube_fir/tube_theta_fir; 
tube_r_sec=s_tube_sec/tube_theta_sec; 
R_6_t2=rotx(-tube_theta_fir)*rotx(-tube_theta_sec); 
R_0_t2=R_0_6*R_6_t2;
draw_t_theta_fir=linspace(0,tube_theta_fir,npts/2);
draw_t_theta_sec=linspace(0,tube_theta_sec,npts/2);
if tube_theta_fir==0
    p_6_t2_fir=180*z_axis*ones(1,npts/2)+[zeros(1,npts/2);zeros(1,npts/2);linspace(0,s_tube_fir,npts/2)];
    p_6_t2_sec=p_6_t2_fir(:,npts/2)*ones(1,npts/2)+rotx(-tube_theta_fir)*[zeros(1,npts/2);tube_r_sec-cos(draw_t_theta_sec)*tube_r_sec;sin(draw_t_theta_sec)*tube_r_sec];
elseif tube_theta_sec==0
    p_6_t2_fir=180*z_axis*ones(1,npts/2)+[zeros(1,npts/2);tube_r_fir-cos(draw_t_theta_fir)*tube_r_fir;sin(draw_t_theta_fir)*tube_r_fir];
    p_6_t2_sec=p_6_t2_fir(:,npts/2)*ones(1,npts/2)+rotx(-tube_theta_fir)*[zeros(1,npts/2);zeros(1,npts/2);linspace(0,s_tube_sec,npts/2)];
else
p_6_t2_fir=180*z_axis*ones(1,npts/2)+[zeros(1,npts/2);tube_r_fir-cos(draw_t_theta_fir)*tube_r_fir;sin(draw_t_theta_fir)*tube_r_fir];
p_6_t2_sec=p_6_t2_fir(:,npts/2)*ones(1,npts/2)+rotx(-tube_theta_fir)*[zeros(1,npts/2);tube_r_sec-cos(draw_t_theta_sec)*tube_r_sec;sin(draw_t_theta_sec)*tube_r_sec];
end
p_6_t2=[p_6_t2_fir,p_6_t2_sec];
p_0_t2=p_0_o6*ones(1,npts)+R_0_6*p_6_t2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%


if strcmp(draw_flag,'draw')
    
    %link0
    cylinderhandle1=draw_cylinder(figure_handle,p_0_o0,rad_link0*R_0_0(:,1),l_link0*R_0_0(:,3),'m',transprancy,'draw');
    %link1
    cylinderhandle2=draw_cylinder(figure_handle,p_0_o1,rad_link1a*R_0_1(:,1),-l_link1a*R_0_1(:,3),'y',transprancy,'draw');
    cylinderhandle3=draw_cylinder(figure_handle,p_0_o1b,rad_link1b*R_0_1(:,3),-l_link2b*R_0_1(:,2),'y',transprancy,'draw');
    %link2
    cylinderhandle4=draw_cylinder(figure_handle,p_0_o2,rad_link2a*R_0_2*x_axis,-(d2-150)*R_0_2*z_axis,'g',transprancy,'draw');
    cylinderhandle5=draw_cylinder(figure_handle,p_0_o2b,rad_link2b*R_0_2*x_axis,-(d2-150)*R_0_2*z_axis,'g',transprancy,'draw');
    cube1=draw_hexahedron(figure_handle,cubevertex1,'g',transprancy,'draw');
    %link3
    cylinderhandle6=draw_cylinder(figure_handle,p_0_o3a,rad_link3a*R_0_3*x_axis,-300*R_0_3*z_axis,'b',transprancy,'draw');
    cylinderhandle7=draw_cylinder(figure_handle,p_0_o3b,rad_link3b*R_0_3*x_axis,(30+114.4)*R_0_3*y_axis,'b',transprancy,'draw');
    %link4
    cylinderhandle8=draw_cylinder(figure_handle,p_0_o4,rad_link4*R_0_4*x_axis,(-330.6-(590-445))*R_0_4*z_axis,'c',transprancy,'draw');
    cylinderhandle9=draw_cylinder(figure_handle,p_0_o4b,rad_link4*R_0_4*x_axis,-116*R_0_4*y_axis,'c',transprancy,'draw');
    %link5
    cylinderhandle10=draw_cylinder(figure_handle,p_0_o5a,rad_link5*R_0_5*x_axis,-70*R_0_5*z_axis,'k',transprancy,'draw');
    cylinderhandle11=draw_cylinder(figure_handle,p_0_o5,rad_link5b*R_0_5*x_axis,65*R_0_5*y_axis,'k',transprancy,'draw');
    %link6
    cylinderhandle12=draw_cylinder(figure_handle,p_0_o6,rad_link6*R_0_6*x_axis,-25*R_0_6*z_axis,'r',transprancy,'draw');
    cube2=draw_hexahedron(figure_handle,cubevertex2,'r',transprancy,'draw');
    %     cylinderhandle13=draw_cylinder(figure_handle,p_0_o6b,rad_link6b*R_0_6*x_axis,400*R_0_6*z_axis,'r',transprancy,'draw');% Ö±µÄÔ²ÖùĞÎtrocar
    hold on
    tubehandle=plot3(p_0_t2(1,:),p_0_t2(2,:),p_0_t2(3,:),'g','LineWidth',5);
    
    cylinderhandle=[cylinderhandle1;cylinderhandle2;cylinderhandle3;cylinderhandle4;cylinderhandle5;cylinderhandle6;cylinderhandle7;cylinderhandle8;cylinderhandle9;cylinderhandle10;cylinderhandle11;cylinderhandle12];
    cubehandle=[cube1;cube2];
    
    %continuum
    cylinder_handles=zeros(4,18); linehandles=zeros(1,120);gripperhandle=zeros(1,3); coo_handle=zeros(6,100);
    [cylinder_handles, linehandles, gripperhandle, coo_handle]=draw_continuum(figure_handle,p_0_t2(:,npts),R_0_t2,'draw',[delta,theta],cylinder_handles,linehandles,gripperhandle,coo_handle);
    
    axis equal;
    %     axis([-500 500 -200 1500 0 1200]);
    grid on
    view(60,30)
    xlabel('x(mm)')
    ylabel('y(mm)')
    zlabel('z(mm)')
    zoom(2.5);
    
end

if strcmp(draw_flag,'update')
    
    cylinderhandle1=draw_cylinder(figure_handle,p_0_o0,rad_link0*R_0_0(:,1),l_link0*R_0_0(:,3),'m',transprancy,'update',denso_cylinderhandle(1,:));
    
    cylinderhandle2=draw_cylinder(figure_handle,p_0_o1,rad_link1a*R_0_1(:,1),-l_link1a*R_0_1(:,3),'y',transprancy,'update',denso_cylinderhandle(2,:));
    
    cylinderhandle3=draw_cylinder(figure_handle,p_0_o1b,rad_link1b*R_0_1(:,3),-l_link2b*R_0_1(:,2),'y',transprancy,'update',denso_cylinderhandle(3,:));
    
    
    cylinderhandle4=draw_cylinder(figure_handle,p_0_o2,rad_link2a*R_0_2*x_axis,-(d2-150)*R_0_2*z_axis,'g',transprancy,'update',denso_cylinderhandle(4,:));
    
    cylinderhandle5=draw_cylinder(figure_handle,p_0_o2b,rad_link2b*R_0_2*x_axis,-(d2-150)*R_0_2*z_axis,'g',transprancy,'update',denso_cylinderhandle(5,:));
    
    cube1=draw_hexahedron(figure_handle,cubevertex1,'g',transprancy,'update',denso_cubehandle(1,:));
    
    
    cylinderhandle6=draw_cylinder(figure_handle,p_0_o3a,rad_link3a*R_0_3*x_axis,-300*R_0_3*z_axis,'b',transprancy,'update',denso_cylinderhandle(6,:));
    
    cylinderhandle7=draw_cylinder(figure_handle,p_0_o3b,rad_link3b*R_0_3*x_axis,(30+114.4)*R_0_3*y_axis,'b',transprancy,'update',denso_cylinderhandle(7,:));
    
    
    cylinderhandle8=draw_cylinder(figure_handle,p_0_o4,rad_link4*R_0_4*x_axis,(-330.6-(590-445))*R_0_4*z_axis,'c',transprancy,'update',denso_cylinderhandle(8,:));
    
    cylinderhandle9=draw_cylinder(figure_handle,p_0_o4b,rad_link4*R_0_4*x_axis,-116*R_0_4*y_axis,'c',transprancy,'update',denso_cylinderhandle(9,:));
    
    
    cylinderhandle10=draw_cylinder(figure_handle,p_0_o5a,rad_link5*R_0_5*x_axis,-70*R_0_5*z_axis,'k',transprancy,'update',denso_cylinderhandle(10,:));
    
    cylinderhandle11=draw_cylinder(figure_handle,p_0_o5,rad_link5b*R_0_5*x_axis,65*R_0_5*y_axis,'k',transprancy,'update',denso_cylinderhandle(11,:));
    
    cylinderhandle12=draw_cylinder(figure_handle,p_0_o6,rad_link6*R_0_6*x_axis,-25*R_0_6*z_axis,'r',transprancy,'update',denso_cylinderhandle(12,:));
    
    cube2=draw_hexahedron(figure_handle,cubevertex2,'r',transprancy,'update',denso_cubehandle(2,:));
    
    
    % cylinderhandle13=draw_cylinder(figure_handle,p_0_o6b,rad_link6b*R_0_6*x_axis,400*R_0_6*z_axis,'r',transprancy,'update',denso_cylinderhandle(13,:));
    set(tubehandle,'XData',p_0_t2(1,:),'YData',p_0_t2(2,:),'ZData',p_0_t2(3,:));
    
    [cylinder_handles, linehandles, gripperhandle, coo_handle]=draw_continuum(figure_handle,p_0_t2(:,npts),R_0_t2,'update',[delta,theta],denso_chandle,denso_lhandle,denso_ghandle,coo_handle);
    
    cylinderhandle=[cylinderhandle1;cylinderhandle2;cylinderhandle3;cylinderhandle4;cylinderhandle5;cylinderhandle6;cylinderhandle7;cylinderhandle8;cylinderhandle9;cylinderhandle10;cylinderhandle11;cylinderhandle12];
    cubehandle=[cube1;cube2];
end



end

