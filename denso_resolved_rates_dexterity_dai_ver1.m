% simiplified denso_continuum control with arbitary circular tube shape to verify dexterity
% by Zhengchen Dai 20150612
% initialization method changed: 仅仅定位spherical wrist交叉点的位置
close
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
[mov1_p_o0,q_c1]=initialize_denso_configuration_with_rcm_target(p_0_wristcenter);
[q_c1,tube_para1]=rcm_target_resolved_rates(q_c1,mov1_p_o0,p_0_rcm,R_0_rcm,tube_para1);
q_c1_origin=q_c1;
tube_para1_origin=tube_para1;

[p_c1,R_c1,p_rcm1,R_rcm1,J_grip1,J_denso1,J_rcm_p1]=denso_kinematics_dexterity_verify_dai(mov1_p_o0,q_c1,tube_para1);
p_rcm_origin1=p_0_rcm;

% set simulation parameters
eps=0.1;%paper中说是0.02
lamda=0.05;%paper中说是0.02
d_t=0.002;
error_p_desire=0.2;
error_r_desire=0.03;
v_lim=15;
w_lim=0.6;
v_lim_low=0.5;
w_lim_low=0.05;
sphere_r=190; % 模拟腹腔罩子半径
steplimit=3000000;
badcount_std=1e4;
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
p_t1=p_cube(:,1);
R_t1=rotx(0);
target_handle=draw_coordinate_system2(1,30,R_t1,p_t1,'rgb','draw');
%%%%%%%

% for vertex_m=1:(size(p_cube,2))
vertex_m=4;
    i_record=0;
    i_singular=0;
    i_RCM_violate=0;
    p_norm_t1=p_cube(:,vertex_m);
    p_t1=p_norm_t1;
%     for beta=0:15/180*pi:(pi-15/180*pi)
%         for alpha=0:15/180*pi:2*pi
%             if alpha==2*pi
%                 break;
%             end
%             for gamma=0:15/180*pi:2*pi
%                 if gamma==2*pi
%                     break;
%                 end
beta=50/180*pi;alpha=140/180*pi;gamma=-50/180*pi;
                i_ran=0;
                targetreach_sign=0;
                randtarget_sign=0;
                R_norm_t1=rotx(alpha)*roty(beta)*rotz(gamma);
%                 R_norm_t1=rotx(pi/2)*roty(0)*rotz(0);
                R_t1=R_norm_t1;
                
                %%%%% draw sentences
                target_handle=draw_coordinate_system2(1,30,R_t1,p_t1,'rgb','update',target_handle);
                %%%%%
                
                while i_ran~=10
                    j=0;
                    badcount=0;
                    if targetreach_sign==1
%                         gamma_record(vertex_m,alpha/(15/180*pi)+1,beta/(15/180*pi)+1,gamma/(15/180*pi)+1)=1;
                        display('reach!!!!');
                        break;
                    end
                    if randtarget_sign==0
                        p_t1=p_norm_t1;
                        R_t1=R_norm_t1;
                        
%                         %%%%%%% draw sentences
%                         target_handle=draw_coordinate_system2(1,30,R_t1,p_t1,'rgb','update',target_handle);
%                         %%%%%%%
                        
                    elseif randtarget_sign==1
                        sphere_rand_alpha=(rand*2-1)*pi;
                        sphere_rand_beta=(rand*2-1)*pi;
                        sphere_rand_r=rand*sphere_r;
                        p_t1=[sphere_rand_r*cos(sphere_rand_alpha)*sin(sphere_rand_beta);sphere_rand_r*cos(sphere_rand_alpha)*cos(sphere_rand_beta);sphere_rand_r*sin(sphere_rand_alpha)];
                        R_t1=rotx((rand*2-1)*pi)*roty((rand*2-1)*pi)*rotz((rand*2-1)*pi);
                        i_ran=i_ran+1;
                        
%                         %%%%%%% draw sentences
%                         target_handle=draw_coordinate_system2(1,30,R_t1,p_t1,'rgb','update',target_handle);
%                         %%%%%%%
                        
                    end
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
                            if randtarget_sign==0
                                targetreach_sign=1;
                                break;
                            elseif randtarget_sign==1
                                randtarget_sign=0;
                                break;
                            end
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
                        [Ugrip,Sgrip,Vgrip]=svd(J_grip1);
                        i_record=i_record+1;
                        record_Srcm(:,:,i_record)=Srcm;
                        record_Sdenso(:,:,i_record)=Sdenso;
                        record_Sgrip(:,:,i_record)=Sgrip;
                        record_Srt(:,:,i_record)=Srt;
                        record_q_c1(:,i_record)=q_c1;
                        record_tube_para1(:,i_record)=tube_para1';
                        record_p_t1(:,i_record)=p_t1;
                        record_R_t1(:,:,i_record)=R_t1;
%                         if Sdenso(6,6)<= eps % paper中是0.02
%                             if Srt(6,6)<=eps
%                             J_rt_plus1 = transpose(J_rt1)/(J_rt1*transpose(J_rt1)+lamda*eye(6));
%                             display('denso 1 singularity');
%                             badcount=badcount+1;
%                             i_singular=i_singular+1;
%                             if mod(i_singular,100)==0
%                             %%%%%%% draw sentences
%                             target_handle=draw_coordinate_system2(1,30,R_t1,p_t1,'rgb','update',target_handle);
%                             [denso_cylinerhandle1, denso_cubehandle1, denso_chandle1, denso_lhandle1, denso_ghandle1, denso_coo_handle1, tubehandle1]=draw_denso_dexterity_verify_dai(1,mov1_p_o0,q_c1,tube_para1,'update',denso_cylinerhandle1,denso_cubehandle1,denso_chandle1,denso_lhandle1,denso_ghandle1,denso_coo_handle1,tubehandle1);
%                             %%%%%%%
%                             saveas(1,['J_rt singular',num2str(i_singular),'.jpg']);
%                             end
%                             record_Srcm(:,:,i_singular)=Srcm;
%                             record_Sdenso(:,:,i_singular)=Sdenso;
%                             record_Sgrip(:,:,i_singular)=Sgrip;
%                             record_Srt(:,:,i_singular)=Srt;
%                             record_q_c1(:,i_singular)=q_c1;
%                             record_tube_para1(:,i_singular)=tube_para1';
%                             record_p_t1(:,i_singular)=p_t1;
%                             record_R_t1(:,:,i_singular)=R_t1;
                            
                            
%                         else
                            J_rt_plus1=pinv(J_rt1);
%                         end
                        
                        J_total_plus1=(eye(8)-pinv(J_rcm_p1)*J_rcm_p1)*J_rt_plus1;
                        q_dot1=J_total_plus1*x_dot1;
                        q_c1_former=q_c1;
                        q_c1=q_c1+q_dot1*d_t;
                        record_q_dot1(:,:,i_record)=q_dot1;
                        %%%%%%%%%
                        %set joint limits
                        if (q_c1(1)<=-170/180*pi)
                            display('denso1_joint_1_lowerboundary')
                            q_c1=q_c1_former;
                            badcount=badcount_std;
                        end
                        if(q_c1(1)>=170/180*pi)
                            display('denso1_joint_1_upperboundary')
                            q_c1=q_c1_former;
                            badcount=badcount_std;
                        end
                        if (q_c1(2)<=-90/180*pi)
                            display('denso1_joint_2_lowerboundary')
                            q_c1=q_c1_former;
                            badcount=badcount_std;
                        end
                        if(q_c1(2)>=135/180*pi)
                            display('denso1_joint_2_upperboundary')
                            q_c1=q_c1_former;
                            badcount=badcount_std;
                        end
                        if (q_c1(3)<=-80/180*pi)
                            display('denso1_joint_3_lowerboundary')
                            q_c1=q_c1_former;
                            badcount=badcount_std;
                        end
                        if(q_c1(3)>=168/180*pi)
                            display('denso1_joint_3_upperboundary')
                            q_c1=q_c1_former;
                            badcount=badcount_std;
                        end
                        if (q_c1(4)<=-185/180*pi)
                            display('denso1_joint_4_lowerboundary')
                            q_c1=q_c1_former;
                            badcount=badcount_std;
                        end
                        if(q_c1(4)>=185/180*pi)
                            display('denso1_joint_4_upperboundary')
                            q_c1=q_c1_former;
                            badcount=badcount_std;
                        end
                        if (q_c1(5)<=-120/180*pi)
                            display('denso1_joint_5_lowerboundary')
                            q_c1=q_c1_former;
                            badcount=badcount_std;
                        end
                        if(q_c1(5)>=120/180*pi)
                            display('denso1_joint_5_upperboundary')
                            q_c1=q_c1_former;
                            badcount=badcount_std;
                        end
                        if (q_c1(6)<=-2*pi)
                            display('denso1_joint_6_lowerboundary')
                            q_c1=q_c1_former;
                            badcount=badcount_std;
                        end
                        if(q_c1(6)>=2*pi)
                            display('denso1_joint_6_upperboundary')
                            q_c1=q_c1_former;
                            badcount=badcount_std;
                        end
                        if (q_c1(7)<0)
                            q_c1(7)=-q_c1(7);
                            q_c1(8)=q_c1(8)+pi;
                        end
                        if(q_c1(7)>=2*pi/3)
                            display('denso1_joint_7_upperboundary')
                            q_c1=q_c1_former;
                            badcount=badcount_std;
                        end
                        % % % % % % % % % % % % % % % % % % % % % % % % %
                        
                        % cal new RCM position parameter on tube
                        tube_para1=denso_cal_moved_rcmpoint_for_dexerity_verify(mov1_p_o0,q_c1,tube_para1,p_rcm1);
                        
                        [p_c1,R_c1,p_rcm1,R_rcm1,J_grip1,J_denso1,J_rcm_p1]=denso_kinematics_dexterity_verify_dai(mov1_p_o0,q_c1,tube_para1);
                        
                        tube_diff1=p_rcm_origin1-p_rcm1;
                        norm_tube_diff1=norm(tube_diff1);
%                         if Srt(6,6)<= eps
%                             record_norm_tube_diff(i_singular)=norm_tube_diff1;
%                         end
                            record_norm_tube_diff(i_record)=norm_tube_diff1;

                        if norm_tube_diff1>3
                            display('violate RCM point1')
                            i_RCM_violate=i_RCM_violate+1;
%                             %%%%%%% draw sentences
%                             target_handle=draw_coordinate_system2(1,30,R_t1,p_t1,'rgb','update',target_handle);
%                             [denso_cylinerhandle1, denso_cubehandle1, denso_chandle1, denso_lhandle1, denso_ghandle1, denso_coo_handle1, tubehandle1]=draw_denso_dexterity_verify_dai(1,mov1_p_o0,q_c1,tube_para1,'update',denso_cylinerhandle1,denso_cubehandle1,denso_chandle1,denso_lhandle1,denso_ghandle1,denso_coo_handle1,tubehandle1);
%                             %%%%%%%
%                             saveas(1,['violate RCM',num2str(i_RCM_violate),'.jpg']);
%                             record_violate_q_c1(:,i_RCM_violate)=q_c1;
%                             record_violate_tube_para1(:,i_RCM_violate)=tube_para1';
%                             record_violate_p_t1(:,i_RCM_violate)=p_t1;
%                             record_violate_R_t1(:,:,i_RCM_violate)=R_t1;
                            badcount=badcount_std;
                            q_c1=q_c1_origin;
                            tube_para1=tube_para1_origin;
                            [p_c1,R_c1,p_rcm1,R_rcm1,J_grip1,J_denso1,J_rcm_p1]=denso_kinematics_dexterity_verify_dai(mov1_p_o0,q_c1,tube_para1);
                        end
                        
                        %%%%%%%%%%%%%%%%%%%%%%%%
                        
%                         %%%%%%% draw sentences
%                         if mod(j,300)==0
%                             [denso_cylinerhandle1, denso_cubehandle1, denso_chandle1, denso_lhandle1, denso_ghandle1, denso_coo_handle1, tubehandle1]=draw_denso_dexterity_verify_dai(1,mov1_p_o0,q_c1,tube_para1,'update',denso_cylinerhandle1,denso_cubehandle1,denso_chandle1,denso_lhandle1,denso_ghandle1,denso_coo_handle1,tubehandle1);
%                         end
%                         %%%%%%%
                        
                        if badcount==badcount_std || j==steplimit
                            if randtarget_sign==0
                                randtarget_sign=1;
                                break;
                            elseif randtarget_sign==1
                                randtarget_sign=0;
                                break;
                            end
                        end
                    end
                    
                    %%%%%%% draw sentences
                    [denso_cylinerhandle1, denso_cubehandle1, denso_chandle1, denso_lhandle1, denso_ghandle1, denso_coo_handle1, tubehandle1]=draw_denso_dexterity_verify_dai(1,mov1_p_o0,q_c1,tube_para1,'update',denso_cylinerhandle1,denso_cubehandle1,denso_chandle1,denso_lhandle1,denso_ghandle1,denso_coo_handle1,tubehandle1);
                    %%%%%%%
                    
                end
%             end
%         end
%     end
%     
% end

%% plot record parameter
figure
plot_Srt(1:i_record)=record_Srt(6,6,:);
plot(1:i_record,plot_Srt)
title('record Srt')
figure
plot_Srcm(1:i_record)=record_Srcm(2,2,:);
plot(1:i_record,plot_Srcm)
title('record Srcm')
figure
plot(1:i_record,record_q_dot1(1,:),'r')
hold on
plot(1:i_record,record_q_dot1(2,:),'b')
plot(1:i_record,record_q_dot1(3,:),'c')
plot(1:i_record,record_q_dot1(4,:),'y')
plot(1:i_record,record_q_dot1(5,:),'k')
plot(1:i_record,record_q_dot1(6,:),'g')
plot(1:i_record,record_q_dot1(7,:),'+')
title('record qdot(1:7)')
figure
plot(1:i_record,record_q_dot1(8,:),'*')
title('record qdot8')
figure
plot(1:i_record,record_norm_tube_diff)
title('record norm tube diff')
%% make movie
% vobj=VideoWriter('testavi10','Uncompressed AVI'); %默认帧率是30fps
% vobj=VideoWriter('two_circle_trocar_20170602','MPEG-4');
% vobj.Quality=100; % set the best video quality
% vobj.FrameRate=10; % FPS
% open(vobj);
% writeVideo(vobj,movie_mov);
% close(vobj);

%1231231231














