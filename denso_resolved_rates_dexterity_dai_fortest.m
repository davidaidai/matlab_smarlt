% simiplified denso_continuum control with arbitary circular tube shape to verify dexterity
% by Zhengchen Dai 20170617

% close
clear
clc

% write video
mov=0;
figure('Position',[466,205,1446,791]);
set(gcf,'color',[1,1,1]);

%define double circular tube
s_tube1=400;
s_rcm1=s_tube1; % RCM点开始位于tube末端
s_tube_fir1=200;
s_tube_sec1=s_tube1-s_tube_fir1;
tube_theta_whole1=45/180*pi;
tube_theta_fir1=-30/180*pi;
tube_theta_sec1=tube_theta_whole1-tube_theta_fir1;
tube_para1=[s_rcm1,s_tube_fir1,s_tube_sec1,tube_theta_fir1,tube_theta_sec1];

% define position and orientation of tube end
p_0_rcm=[0;0;190];
R_0_rcm=rotx(180/180*pi);
p_0_wristcenter=[0;0;1000];
mov1_p_o0=[-622.385434027068;-622.385434027068;-282.385315010277];
q_c1=[-0.245806488861565;-0.957294291854478;2.91112048976764;1.43591016699215;0.414529856308439;0.458205817172382;2.03137848471555;-13.6582670485787];
tube_para1=[323.446893787575;200;200;-0.523598775598299;1.30899693899575];
q_c1_origin=q_c1;
tube_para1_origin=tube_para1;

[p_c1,R_c1,p_rcm1,R_rcm1,J_grip1,J_denso1,J_rcm_p1]=denso_kinematics_dexterity_verify_dai(mov1_p_o0,q_c1,tube_para1);
p_rcm_origin1=p_0_rcm;

% set simulation parameters
eps=0.07;%paper中说是0.02
lamda=0.01;%paper中说是0.02
d_t=0.002;
error_p_desire=0.2;
error_r_desire=0.03;
v_lim=15;
w_lim=0.6;
v_lim_low=0.5;
w_lim_low=0.05;
sphere_r=190; % 模拟腹腔罩子半径
steplimit=3000000;
badcount_std=3e3;
gamma_record=zeros(8,25,25,25);
%%%%%%%%%%%%%

i=0;j=0;k=1;
%%%%%%% draw sentences
[denso_cylinerhandle1, denso_cubehandle1, denso_chandle1, denso_lhandle1, denso_ghandle1, denso_coo_handle1,tubehandle1]=draw_denso_dexterity_verify_dai(1,mov1_p_o0,q_c1,tube_para1,'draw');
%%%%%%%

%%%%%%%% draw RCM point, half sphere, target coordinate
plot3(p_rcm_origin1(1),p_rcm_origin1(2),p_rcm_origin1(3),'b*','LineWidth',3);
drawsphere(85/180*pi,pi,sphere_r,[0;0;0]);
%%%%%%%%

% cube motion
l_cube=150;
p_cube_start=p_0_rcm+[-75;0;-110];
p_cube1=p_cube_start*ones(1,4)+[0,-l_cube/2,0;l_cube,-l_cube/2,0;l_cube,l_cube/2,0;0,l_cube/2,0]';
p_cube2=p_cube1+[0;0;-100]*ones(1,4);
p_cube=[p_cube1,p_cube2];
cube_plot=p_cube;

%%%%%%% draw coordinate
plot3(cube_plot(1,:),cube_plot(2,:),cube_plot(3,:),'c','LineWidth',1);
%%%%%%%

%%%%%%% draw sentences
p_t1=[121.679308329377;-27.8728566467496;71.5210719738068];
R_t1=[0.6655   -0.3021   -0.6825;
   -0.6310    0.2607   -0.7307;
    0.3987    0.9169   -0.0171];
target_handle=draw_coordinate_system2(1,30,R_t1,p_t1,'rgb','draw');
%%%%%%%

while j<steplimit
        
    error_p1=norm(p_t1-p_c1);
    R_tc1=R_t1*R_c1';% 此时R_tc在世界坐标系下
    error_r1=acos((R_tc1(1,1)+R_tc1(2,2)+R_tc1(3,3)-1)/2);
    
    if abs(error_r1)<=0.01
        r_axis1 = [0;0;0];
    else
        r_axis1 = [R_tc1(3,2)-R_tc1(2,3);R_tc1(1,3)-R_tc1(3,1);R_tc1(2,1)-R_tc1(1,2)]/(2*sin(error_r1));
    end
    
    if error_p1<error_p_desire && error_r1<error_r_desire
        [denso_cylinerhandle1, denso_cubehandle1, denso_chandle1, denso_lhandle1, denso_ghandle1, denso_coo_handle1, tubehandle1]=draw_denso_dexterity_verify_dai(1,mov1_p_o0,q_c1,tube_para1,'update',denso_cylinerhandle1,denso_cubehandle1,denso_chandle1,denso_lhandle1,denso_ghandle1,denso_coo_handle1,tubehandle1);
        
        break;
        
    end
    j=j+1;
    
    if error_p1<=0.5 && error_r1<=0.05
        v1=v_lim_low.*(p_t1 - p_c1)/norm(p_t1 - p_c1);
        w1 = w_lim_low*r_axis1;
    else
        v1=v_lim.*(p_t1 - p_c1)/norm(p_t1 - p_c1);
        w1 = w_lim*r_axis1;
    end
    x_dot1= [v1;w1];
    
    J_rt1=J_grip1*(eye(8)-pinv(J_rcm_p1)*J_rcm_p1);
    
    %sigular value robust
    [Urcm,Srcm,Vrcm]=svd(J_rcm_p1);
    [Udenso,Sdenso,Vdenso] = svd(J_denso1);
    [Urt,Srt,Vrt]=svd(J_rt1);
    record_Srt(:,:,j)=Srt;
    record_tube_para1(:,j)=tube_para1';
%     if Sdenso(6,6)<= eps % paper中是0.02
%     if Srt(6,6)<=eps
%         J_rt_plus1 = transpose(J_rt1)/(J_rt1*transpose(J_rt1)+lamda*eye(6));
%         display('denso 1 singularity');
%     else
        J_rt_plus1=pinv(J_rt1);
%     end
    
    J_total_plus1=(eye(8)-pinv(J_rcm_p1)*J_rcm_p1)*J_rt_plus1;
    q_dot1=J_total_plus1*x_dot1;
    q_c1_former=q_c1;
    q_c1=q_c1+q_dot1*d_t;
    record_q_dot1(:,:,j)=q_dot1;
    % record v_rcm_p, 理论上在不加入singular robust前，v_rcm_p应该时刻保持零
    record_norm_v_rcm_p(j)=norm(J_rcm_p1*q_dot1);
    
    %%%%%%%%%
    %set joint limits
    if (q_c1(1)<=-170/180*pi)
        display('denso1_joint_1_lowerboundary')
    break;
    end
    if(q_c1(1)>=170/180*pi)
        display('denso1_joint_1_upperboundary')
    break;
    end
    if (q_c1(2)<=-90/180*pi)
        display('denso1_joint_2_lowerboundary')
    break;
    end
    if(q_c1(2)>=135/180*pi)
        display('denso1_joint_2_upperboundary')
    break;
    end
    if (q_c1(3)<=-80/180*pi)
        display('denso1_joint_3_lowerboundary')
    break;
    end
    if(q_c1(3)>=168/180*pi)
        display('denso1_joint_3_upperboundary')
    break;
    end
    if (q_c1(4)<=-185/180*pi)
        display('denso1_joint_4_lowerboundary')
    break;
    end
    if(q_c1(4)>=185/180*pi)
        display('denso1_joint_4_upperboundary')
    break;
    end
    if (q_c1(5)<=-120/180*pi)
        display('denso1_joint_5_lowerboundary')
    break;
    end
    if(q_c1(5)>=120/180*pi)
        display('denso1_joint_5_upperboundary')
        break;
    end
    if (q_c1(6)<=-2*pi)
        display('denso1_joint_6_lowerboundary')
        break;
    end
    if(q_c1(6)>=2*pi)
        display('denso1_joint_6_upperboundary')
        break;
    end
    if (q_c1(7)<0)
        q_c1(7)=-q_c1(7);
        q_c1(8)=q_c1(8)+pi;
    end
    if(q_c1(7)>=2*pi/3)
        display('denso1_joint_7_upperboundary')
        break;
    end
    % % % % % % % % % % % % % % % % % % % % % % % % %
    
    % cal new RCM position parameter on tube
    tube_para1=denso_cal_moved_rcmpoint_for_dexerity_verify(mov1_p_o0,q_c1,tube_para1,p_rcm_origin1);
    
    [p_c1,R_c1,p_rcm1,R_rcm1,J_grip1,J_denso1,J_rcm_p1]=denso_kinematics_dexterity_verify_dai(mov1_p_o0,q_c1,tube_para1);
    
    tube_diff1=p_rcm_origin1-p_rcm1;
    norm_tube_diff1=norm(tube_diff1);
    record_norm_tube_diff(j)=norm_tube_diff1;
    if norm_tube_diff1>3
        display('violate RCM point1')
        break;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%% draw sentences
    if mod(j,300)==0
        [denso_cylinerhandle1, denso_cubehandle1, denso_chandle1, denso_lhandle1, denso_ghandle1, denso_coo_handle1, tubehandle1]=draw_denso_dexterity_verify_dai(1,mov1_p_o0,q_c1,tube_para1,'update',denso_cylinerhandle1,denso_cubehandle1,denso_chandle1,denso_lhandle1,denso_ghandle1,denso_coo_handle1,tubehandle1);
    end
    %%%%%%%

end
[denso_cylinerhandle1, denso_cubehandle1, denso_chandle1, denso_lhandle1, denso_ghandle1, denso_coo_handle1, tubehandle1]=draw_denso_dexterity_verify_dai(1,mov1_p_o0,q_c1,tube_para1,'update',denso_cylinerhandle1,denso_cubehandle1,denso_chandle1,denso_lhandle1,denso_ghandle1,denso_coo_handle1,tubehandle1);
%% plot record parameter
figure
plot_Srt(1:j)=record_Srt(6,6,:);
plot(1:j,plot_Srt)
title('record Srt')
figure
plot(1:j,record_q_dot1(1,:),'r')
hold on
plot(1:j,record_q_dot1(2,:),'b')
plot(1:j,record_q_dot1(3,:),'c')
plot(1:j,record_q_dot1(4,:),'y')
plot(1:j,record_q_dot1(5,:),'k')
plot(1:j,record_q_dot1(6,:),'g')
plot(1:j,record_q_dot1(7,:),'+')
title('record qdot(1:7)')
figure
plot(1:j,record_q_dot1(8,:),'*')
title('record qdot8')
figure
plot(1:j,record_norm_v_rcm_p(:))
title('record norm v rcm p')
figure
plot(1:j,record_norm_tube_diff)
title('record norm tube diff')

%% make movie
% vobj=VideoWriter('testavi10','Uncompressed AVI'); %默认帧率是30fps
% vobj=VideoWriter('two_circle_trocar_20170602','MPEG-4');
% vobj.Quality=100; % set the best video quality
% vobj.FrameRate=10; % FPS
% open(vobj);
% writeVideo(vobj,movie_mov);
% close(vobj);
















