% simiplified denso_continuum control with arbitary circular tube shape to verify dexterity
% by Zhengchen Dai 20150622
% initialization method changed: 仅仅定位spherical wrist交叉点的位置
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
[mov1_p_o0,q_c1]=initialize_denso_configuration_with_rcm_target(p_0_wristcenter);
[q_c1,tube_para1]=rcm_target_resolved_rates(q_c1,mov1_p_o0,p_0_rcm,R_0_rcm,tube_para1);
% q_c1=[-0.716236043028513;-0.236164102694094;2.52105526426432;1.36220527433854;1.18075517314056;-0.817985975190416;0.01;1.5];
% tube_para1=[362.414543160652,200,200,-0.523598775598299,1.30899693899575];
q_c1_origin=q_c1;
tube_para1_origin=tube_para1;

% set simulation parameters
eps=0.05;
lamda=0.005;
d_t=1e-3;
error_p_desire=0.2;
error_r_desire=1/180*pi;
v_lim_bound=[50,10];% upper bound and lower bound
w_lim_bound=[150/180*pi,30/180*pi];
v_rcm_p_lim=1/d_t;
sphere_r=190; % 模拟腹腔罩子半径
steplimit=3000000;
badcount_std=1e4;
gamma_record=zeros(8,25,25,25);
i=0;j=0;k=1;
singular_sign=0;
%%%%%%%%%%%%%

[p_c1,R_c1,p_rcm1,R_rcm1,J_grip1,J_denso1,J_rcm_p1]=denso_kinematics_dexterity_verify_dai(mov1_p_o0,q_c1,tube_para1);
p_rcm_origin1=p_0_rcm;
tube_diff1=p_rcm_origin1-p_rcm1;
v_rcm_p1=v_rcm_p_lim*tube_diff1;
v_rcm_p1=v_rcm_p1-(v_rcm_p1'*R_rcm1(:,3))*R_rcm1(:,3);
if v_rcm_p1 >= v_lim_bound(1)/2
    v_rcm_p1=v_lim_bound(1)/2;
end
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
vertex_m=7;
    i_record=0;
    i_singular=0;
    i_RCM_violate=0;
    p_norm_t1=p_cube(:,vertex_m);
    p_t1=p_norm_t1;
%     for beta=0/180*pi:30/180*pi:(pi-30/180*pi)
%         for alpha=0:30/180*pi:(2*pi-30/180*pi)
%             for gamma=0:30/180*pi:(2*pi-30/180*pi)
%                 singular_sign=0;
%                 q_c1=[-0.549765924054457;-0.262651594588122;2.10989094467760;0.819158840351031;1.21046383082078;-0.0428033484084231;0.785398163397448;-1.57079632679490];
%                 tube_para1=[400,200,200,-0.523598775598299,1.30899693899575];
%                 [p_c1,R_c1,p_rcm1,R_rcm1,J_grip1,J_denso1,J_rcm_p1]=denso_kinematics_dexterity_verify_dai(mov1_p_o0,q_c1,tube_para1);
% beta=0;alpha=90/180*pi;gamma=0;
% beta=-30/180*pi;alpha=90/180*pi;gamma=30/180*pi;
% beta=-50/180*pi;alpha=150/180*pi;gamma=-10/180*pi;
% beta=50/180*pi;alpha=140/180*pi;gamma=-50/180*pi;
% beta=0/180*pi;alpha=270/180*pi;gamma=90/180*pi; % theta=0导致algorithm singularity，vetex 4
% beta=0/180*pi;alpha=0/180*pi;gamma=90/180*pi; % 其他原因导致algorithm singularity，但无法到达， vetex 7
beta=0/180*pi;alpha=60/180*pi;gamma=30/180*pi; % 其他原因导致algorithm singularity，可到达，vetex7 
                i_ran=0;
                targetreach_sign=0;
                randtarget_sign=0;
                R_norm_t1=rotx(alpha)*roty(beta)*rotz(gamma);
%                 R_norm_t1=rotx(pi/2)*roty(0)*rotz(0);
                R_t1=R_norm_t1;
                
                %%%%% draw sentences
                target_handle=draw_coordinate_system2(1,30,R_t1,p_t1,'rgb','update',target_handle);
                %%%%%
                
                while i_ran~=1

                    if targetreach_sign==1
%                         gamma_record(vertex_m,alpha/(15/180*pi)+1,beta/(15/180*pi)+1,gamma/(15/180*pi)+1)=1;
                        display('reach!!!!');
%                         if singular_sign==1
%                             mmmm=1;
%                         end
                        break;
                    end
                    if randtarget_sign==0
                        p_t1=p_norm_t1;
                        R_t1=R_norm_t1;
                        
                        %%%%%%% draw sentences
                        target_handle=draw_coordinate_system2(1,30,R_t1,p_t1,'rgb','update',target_handle);
                        %%%%%%%
                        
                    elseif randtarget_sign==1
                        sphere_rand_alpha=(rand*2-1)*pi;
                        sphere_rand_beta=(rand*2-1)*pi;
                        sphere_rand_r=rand*sphere_r;
                        p_t1=[sphere_rand_r*cos(sphere_rand_alpha)*sin(sphere_rand_beta);sphere_rand_r*cos(sphere_rand_alpha)*cos(sphere_rand_beta);sphere_rand_r*sin(sphere_rand_alpha)];
                        R_t1=rotx((rand*2-1)*pi)*roty((rand*2-1)*pi)*rotz((rand*2-1)*pi);
                        if j>=100 % 防止连续产生的随机目标点均使得关节处于limit，而达不到产生disturbance的效果
                        i_ran=i_ran+1;
                        end
                        %%%%%%% draw sentences
                        target_handle=draw_coordinate_system2(1,30,R_t1,p_t1,'rgb','update',target_handle);
                        %%%%%%%
                        
                    end
                    
                    j=0;
                    badcount=0;
                    
                    while j<steplimit
                        
                        error_p1=norm(p_t1-p_c1);
                        R_tc1=R_t1*R_c1';% 此时R_tc在世界坐标系下
                        error_r1=acos((R_tc1(1,1)+R_tc1(2,2)+R_tc1(3,3)-1)/2);
                        
                        r_axis1 = [R_tc1(3,2)-R_tc1(2,3);R_tc1(1,3)-R_tc1(3,1);R_tc1(2,1)-R_tc1(1,2)]/(2*sin(error_r1));

                        
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
                        
%                         if j==500%4327
%                             jjj=1;
%                         end
                        
                        % velocity regulation
                        if error_p1 >=v_lim_bound(1)
                            v_lim=v_lim_bound(1);
                        elseif error_p1 <v_lim_bound(1) && error_p1 > v_lim_bound(2)
                            v_lim=error_p1;
                        elseif error_p1 <=v_lim_bound(2)
                            v_lim=v_lim_bound(2);
                        end
                        if error_r1 >=w_lim_bound(1)
                            w_lim=w_lim_bound(1);
                        elseif error_r1 <w_lim_bound(1) && error_r1 > w_lim_bound(2)
                            w_lim=error_r1;
                        elseif error_r1 <=w_lim_bound(2)
                            w_lim=w_lim_bound(2);
                        end
                        
                        if error_p1>=v_lim_bound(1)
                            tv=(error_p1-v_lim_bound(1))/v_lim_bound(1)+(log(v_lim_bound(1))-log(v_lim_bound(2)))+v_lim_bound(2)/v_lim_bound(2);
                        elseif error_p1 <v_lim_bound(1) && error_p1 > v_lim_bound(2)
                            tv=(log(v_lim)-log(v_lim_bound(2)))+v_lim_bound(2)/v_lim_bound(2);
                        elseif error_p1 <= v_lim_bound(2)
                            tv=error_p1/v_lim_bound(2);
                        end
                        
                        if error_r1>=w_lim_bound(1)
                            tw=(error_r1-w_lim_bound(1))/w_lim_bound(1)+(log(w_lim_bound(1))-log(w_lim_bound(2)))+w_lim_bound(2)/w_lim_bound(2);
                        elseif error_r1 <w_lim_bound(1) && error_r1 > w_lim_bound(2)
                            tw=(log(w_lim)-log(w_lim_bound(2)))+w_lim_bound(2)/w_lim_bound(2);
                        elseif error_r1 <= w_lim_bound(2)
                            tw=error_r1/w_lim_bound(2);
                        end
                        
                        w_lim_c=tw/tv;
                        v_lim_c=tv/tw;
                        
                        w_lim=w_lim_c*w_lim;
                        if w_lim>= w_lim_bound(1)
                            w_lim=w_lim_bound(1);
                        end

                        v_lim=v_lim_c*v_lim;
                        if v_lim>= v_lim_bound(1)
                            v_lim=v_lim_bound(1);
                        end
                                                
                        v1=v_lim.*(p_t1 - p_c1)/norm(p_t1 - p_c1);
                        w1 = w_lim*r_axis1;
                        
                        x_dot1= [v1;w1];
                         
                        J_rt1=J_grip1*(eye(8)-pinv(J_rcm_p1)*J_rcm_p1);
                                                
                        %sigular value robust
                        [Urcm,Srcm,Vrcm]=svd(J_rcm_p1);
                        [Udenso,Sdenso,Vdenso] = svd(J_denso1);
                        [Urt,Srt,Vrt]=svd(J_rt1);
                        [Ugrip,Sgrip,Vgrip]=svd(J_grip1);
                        i_record=i_record+1;
                        record_v_lim(:,i_record)=v_lim;
                        record_w_lim(:,i_record)=w_lim;
                        record_Srcm(:,:,i_record)=Srcm;
                        record_Sdenso(:,:,i_record)=Sdenso;
                        record_Sgrip(:,:,i_record)=Sgrip;
                        record_Srt(:,:,i_record)=Srt;
                        record_q_c1(:,i_record)=q_c1;
                        record_tube_para1(:,i_record)=tube_para1';
                        record_p_t1(:,i_record)=p_t1;
                        record_R_t1(:,:,i_record)=R_t1;
                        record_error_p1(:,i_record)=error_p1;
                        record_error_r1(:,i_record)=error_r1;
                        
                        if Srt(6,6)<=eps
%                             J_rt_plus1 = transpose(J_rt1)/(J_rt1*transpose(J_rt1)+lamda*eye(6));
                            dpl=(1-(Srt(6,6)/eps)^2)*lamda;
%                             J_rt_plus1 = transpose(J_rt1)/(J_rt1*transpose(J_rt1)+dpl*eye(6));
                            J_rt_plus1 = transpose(J_rt1)/(J_rt1*transpose(J_rt1)+1e-5*eye(6)+dpl*Urt(:,6)*Urt(:,6)');
                            display('denso 1 singularity');
                            badcount=badcount+1;
                            singular_sign=1;
                        else
                            J_rt_plus1=pinv(J_rt1);
                        end
                        q_dot1=pinv(J_rcm_p1)*v_rcm_p1+J_rt_plus1*(x_dot1-J_grip1*pinv(J_rcm_p1)*v_rcm_p1);
                        record_q_dot1(:,:,i_record)=q_dot1;
                        r=2.5/24*30;
                        theta=q_c1(7);
                        delta=q_c1(8);
                        J_cq_psi=[-r*cos(delta) r*theta*sin(delta); r*sin(delta) r*theta*cos(delta)];
                        record_cq_dot(:,i_record)=J_cq_psi*q_dot1(7:8);
%                            record1(:,:,i_record)=J_rt_plus1;
%                            record2(:,:,i_record)=J_grip1;
%                            record3(:,i_record)=pinv(J_rcm_p1)*v_rcm_p1;

% if Srt(6,6)<=eps
%     display('denso 1 Jrt singularity');
%     if Sgrip(6,6)<=eps
%         display('denso 1 Jgrip singularity');
%         dpl_grip=(1-(Sgrip(6,6)/eps)^2)*lamda;
%         J_grip1_plus=J_grip1'/(J_grip1*J_grip1'+dpl_grip*eye(6));
%     else
%         J_grip1_plus=pinv(J_grip1);
%     end
%     J_rt_plus1=J_grip1_plus;
%     J_total_plus1=(eye(8)-pinv(J_rcm_p1)*J_rcm_p1)*J_rt_plus1;
%     q_dot1=pinv(J_rcm_p1)*v_rcm_p1+J_total_plus1*x_dot1;
% else
%     J_rt_plus1=pinv(J_rt1);
%     q_dot1=pinv(J_rcm_p1)*v_rcm_p1+J_rt_plus1*(x_dot1-J_grip1*pinv(J_rcm_p1)*v_rcm_p1);
% %     J_total_plus1=(eye(8)-pinv(J_rcm_p1)*J_rcm_p1)*J_rt_plus1;
% %     q_dot1=pinv(J_rcm_p1)*v_rcm_p1+J_total_plus1*(x_dot1-J_grip1*pinv(J_rcm_p1)*v_rcm_p1);
% end


q_c1_former=q_c1;
q_c1=q_c1+q_dot1*d_t;

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
                        tube_para1=denso_cal_moved_rcmpoint_for_dexerity_verify(mov1_p_o0,q_c1,tube_para1,p_rcm_origin1);
                        
                        [p_c1,R_c1,p_rcm1,R_rcm1,J_grip1,J_denso1,J_rcm_p1]=denso_kinematics_dexterity_verify_dai(mov1_p_o0,q_c1,tube_para1);
                        
                        tube_diff1=p_rcm_origin1-p_rcm1;
                        v_rcm_p1=v_rcm_p_lim*tube_diff1;
                        v_rcm_p1=v_rcm_p1-(v_rcm_p1'*R_rcm1(:,3))*R_rcm1(:,3);
                        if norm(v_rcm_p1) >= v_lim_bound(1)/2
                            v_rcm_p1=v_lim_bound(1)/2*v_rcm_p1/norm(v_rcm_p1);
                        end
                        norm_tube_diff1=norm(tube_diff1);
                        
                        record_norm_tube_diff(i_record)=norm_tube_diff1;
                        record_norm_v_rcm_p(i_record)=norm(v_rcm_p1);

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
                        
                        %%%%%%% draw sentences
%                         if mod(j,100)==0
%                             [denso_cylinerhandle1, denso_cubehandle1, denso_chandle1, denso_lhandle1, denso_ghandle1, denso_coo_handle1, tubehandle1]=draw_denso_dexterity_verify_dai(1,mov1_p_o0,q_c1,tube_para1,'update',denso_cylinerhandle1,denso_cubehandle1,denso_chandle1,denso_lhandle1,denso_ghandle1,denso_coo_handle1,tubehandle1);
%                         end
                        %%%%%%%
                        
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
% figure
% plot_Srcm(1:i_record)=record_Srcm(2,2,:);
% plot(1:i_record,plot_Srcm)
% title('record Srcm')
figure
plot(1:i_record,record_q_dot1(1,1:i_record),'r')
hold on
plot(1:i_record,record_q_dot1(2,1:i_record),'b')
plot(1:i_record,record_q_dot1(3,1:i_record),'c')
plot(1:i_record,record_q_dot1(4,1:i_record),'y')
% [~,handle_5,handle_7]=plotyy(1:i_record,record_q_dot1(5,1:i_record),1:i_record,record_cq_dot(1,:));
% [~,handle_6,handle_8]=plotyy(1:i_record,record_q_dot1(6,1:i_record),1:i_record,record_cq_dot(2,:));
% set(handle_5,'Color','k');
% set(handle_6,'Color','g');
% set(handle_7,'LineStyle','-.');
% set(handle_8,'LineStyle','--');
plot(1:i_record,record_q_dot1(5,1:i_record),'k')
plot(1:i_record,record_q_dot1(6,1:i_record),'g')
plot(1:i_record,record_cq_dot(1,:),'-.')
plot(1:i_record,record_cq_dot(2,:),'r-.')
title('record qdot(1:8)')
% figure
% plot(2:i_record,record_q_dot1(8,2:i_record))
% title('record qdot8')
figure
plot(1:i_record,record_norm_tube_diff)
ylim([0 3]);
title('record norm tube diff')
figure
plot(1:i_record,record_error_p1)
title('record error p1')
figure
plot(1:i_record,record_error_r1)
title('record error r1')
figure
plot(1:i_record,record_v_lim)
title('record norm v')
figure
plot(1:i_record,record_w_lim)
title('record norm w')
figure
plot(1:i_record,record_norm_v_rcm_p)
title('record norm v rcm p')
figure
plot_Sgrip(1:i_record)=record_Sgrip(6,6,:);
plot(1:i_record,plot_Sgrip)
title('record Sgrip(6,6)')

%% make movie
% vobj=VideoWriter('testavi10','Uncompressed AVI'); %默认帧率是30fps
% vobj=VideoWriter('two_circle_trocar_20170602','MPEG-4');
% vobj.Quality=100; % set the best video quality
% vobj.FrameRate=10; % FPS
% open(vobj);
% writeVideo(vobj,movie_mov);
% close(vobj);















