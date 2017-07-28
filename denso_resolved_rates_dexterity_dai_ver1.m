% simiplified denso_continuum control with arbitary circular tube shape to verify dexterity
% by Zhengchen Dai
close
clear
clc

%% set simulation parameters
eps=0.05;
lamda=0.05;
d_t=1e-3;
error_p_desire=0.2;
error_r_desire=1/180*pi;
v_lim_bound=[50,20];% upper bound and lower bound
w_lim_bound=[150/180*pi,100/180*pi];
v_rcm_p_lim=1/d_t;
sphere_r=190; % 模拟腹腔罩子半径
steplimit=600000; % 相当于smarlt跑十分钟
badcount_std=1e4;
i=0;k=1;
i_record=0;
i_RCM_violate=0;
i_tube=0;
s_tube1=400;
tube_theta_whole1=45/180*pi;
p_0_rcm=[0;0;190];
R_0_rcm=rotx(180/180*pi);
p_0_wristcenter=[0;0;1000];
record_gamma_degree=zeros(6,1e3,50);
record_gamma_q_c1=zeros(8,1e3,50);
record_gamma_tube_para1=zeros(5,1e3,50);
record_gamma_p_t1=zeros(3,1e3,50);
record_gamma_R_t1=zeros(3,3,1e3,50);
i_singular1=0;
i_singular2=0;
% cube motion
l_cube=150;
p_cube_start=p_0_rcm+[-75;0;-110];
p_cube1=p_cube_start*ones(1,4)+[0,-l_cube/2,0;l_cube,-l_cube/2,0;l_cube,l_cube/2,0;0,l_cube/2,0]';
p_cube2=p_cube1+[0;0;-100]*ones(1,4);
p_cube_center=p_0_rcm+[0;0;-110-100/2];
p_cube=[p_cube1,p_cube2,p_cube_center];
cube_plot=p_cube;

% %% write video
% mov=0;
% figure('Position',get(0,'screensize'));
% % set(gcf,'color',[1,1,1]);
% % axis equal;
% axis tight;

%% define double circular tube
for s_tube_fir1=0:50:350
%     s_tube_fir1=50;
    for tube_theta_fir1=-30/180*pi:30/180*pi:0/180*pi
        i_gamma_total=0;
        i_gamma=0;
        if s_tube_fir1==0
            tube_theta_fir1=0;
        end
        i_tube=i_tube+1;
        s_rcm1=s_tube1; % RCM点开始位于tube末端
        s_tube_sec1=s_tube1-s_tube_fir1;
        tube_theta_sec1=tube_theta_whole1-tube_theta_fir1;
        tube_para1=[s_rcm1,s_tube_fir1,s_tube_sec1,tube_theta_fir1,tube_theta_sec1];

%% define position and orientation of tube end

[mov1_p_o0,q_c1]=initialize_denso_configuration_with_rcm_target(p_0_wristcenter);
[q_c1,tube_para1]=rcm_target_resolved_rates(q_c1,mov1_p_o0,p_0_rcm,R_0_rcm,tube_para1);
% q_c1=[0.0813289811105561;0.194764461193291;1.99109029223081;-0.388811394753712;0.773476809438931;2.65144686444801;1.71872460393387;4.66018521416587];
% tube_para1=[191.343314489471;100;300;-0.261799387799149;1.04719755119660];
q_c1_origin=q_c1;
tube_para1_origin=tube_para1;

%% draw sentences
if i_tube==1
[denso_cylinerhandle1, denso_cubehandle1, denso_chandle1, denso_lhandle1, denso_ghandle1, denso_coo_handle1,tubehandle1]=draw_denso_dexterity_verify_dai(1,mov1_p_o0,q_c1,tube_para1,'draw');
%% draw RCM point, half sphere, target coordinate
plot3(p_0_rcm(1),p_0_rcm(2),p_0_rcm(3),'b*','LineWidth',3);
drawsphere(85/180*pi,pi,sphere_r,[0;0;0]);
%% draw cube
plot3(cube_plot(1,:),cube_plot(2,:),cube_plot(3,:),'c','LineWidth',1);
%% draw sentences
p_t1=p_cube(:,1);
R_t1=rotx(0);
target_handle=draw_coordinate_system2(1,30,R_t1,p_t1,'rgb','draw');
end

[p_c1,R_c1,p_rcm1,R_rcm1,J_grip1,J_denso1,J_rcm_p1]=denso_kinematics_dexterity_verify_dai(mov1_p_o0,q_c1,tube_para1);
p_rcm_origin1=p_0_rcm;
tube_diff1=p_rcm_origin1-p_rcm1;
v_rcm_p1=v_rcm_p_lim*tube_diff1;
v_rcm_p1=v_rcm_p1-(v_rcm_p1'*R_rcm1(:,3))*R_rcm1(:,3);
if norm(v_rcm_p1) >= v_lim_bound(1)/2
    v_rcm_p1=v_lim_bound(1)/2*v_rcm_p1/norm(v_rcm_p1);
end

%% loop continue
% for vertex_m=1:(size(p_cube,2))
vertex_m=9;

    p_norm_t1=p_cube(:,vertex_m);
%     p_norm_t1=[24.1955077411972;-9.49868855234999;-185.983900045286];
    p_t1=p_norm_t1;
    for beta=0/180*pi:15/180*pi:(pi-15/180*pi)
        for alpha=0/180*pi:15/180*pi:(2*pi-15/180*pi)
            for gamma=0/180*pi:90/180*pi:(2*pi-90/180*pi)
                if beta~=0/180*pi && (alpha==90/180*pi || alpha==270/180*pi)
                    break;
                end
                display(num2str([s_tube_fir1,tube_theta_fir1/pi*180,vertex_m,beta/pi*180,alpha/pi*180,gamma/pi*180]))
                i_gamma_total=i_gamma_total+1;
                i_ran=0;
                targetreach_sign=0;
                randtarget_sign=0;
                R_norm_t1=roty(beta)*rotx(alpha)*rotz(gamma);
%                 R_norm_t1=[-0.0752453807169914,0.613575358932692,-0.786042881521978;-0.993993342483134,0.0166692556233244,0.108163630746070;0.0794692882814676,0.789460204714324,0.608635537405149];
                R_t1=R_norm_t1;
                
%                 %%%%% draw sentences
%                 target_handle=draw_coordinate_system2(1,30,R_t1,p_t1,'rgb','update',target_handle);
%                 %%%%%
                
                while i_ran~=20
                    if targetreach_sign==1
                        i_gamma=i_gamma+1;
                        record_gamma_degree(:,i_gamma,i_tube)=[s_tube_fir1;tube_theta_fir1;vertex_m;beta/pi*180;alpha/pi*180;gamma/pi*180];
                        record_gamma_q_c1(:,i_gamma,i_tube)=q_c1;
                        record_gamma_tube_para1(:,i_gamma,i_tube)=tube_para1';
                        record_gamma_p_t1(:,i_gamma,i_tube)=p_t1;
                        record_gamma_R_t1(:,:,i_gamma,i_tube)=R_t1;
                        display('reach!!!!');
                        save (['dexterity 20170728 sfir ',num2str(s_tube_fir1)]);
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
                        if j>=100 % 防止连续产生的随机目标点均使得关节处于limit，而达不到产生disturbance的效果
                        i_ran=i_ran+1;
                        end
%                         %%%%%%% draw sentences
%                         target_handle=draw_coordinate_system2(1,30,R_t1,p_t1,'rgb','update',target_handle);
%                         %%%%%%%
                        
                    end
                    
                    j=0;
                    badcount=0;
                    q_c_start=q_c1;
                    tube_para_start=tube_para1;
                    while j<steplimit
                        j=j+1;
                        
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

                        
                     %% velocity regulation
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
                         

                                                
                     %% sigular value robust
                        J_rt1=J_grip1*(eye(8)-pinv(J_rcm_p1)*J_rcm_p1);
                        [Urcm,Srcm,Vrcm]=svd(J_rcm_p1);
                        [Urt,Srt,Vrt]=svd(J_rt1);
                        [Ugrip,Sgrip,Vgrip]=svd(J_grip1);
                        
%                         i_record=i_record+1;
%                         record_v_lim(:,i_record)=v_lim;
%                         record_w_lim(:,i_record)=w_lim;
%                         record_Srcm(:,:,i_record)=Srcm;
%                         record_Sdenso(:,:,i_record)=Sdenso;
%                         record_Sgrip(:,:,i_record)=Sgrip;
%                         record_Srt(:,:,i_record)=Srt;
%                         record_q_c1(:,i_record)=q_c1;
%                         record_tube_para1(:,i_record)=tube_para1';
%                         record_p_t1(:,i_record)=p_t1;
%                         record_R_t1(:,:,i_record)=R_t1;
%                         record_error_p1(:,i_record)=error_p1;
%                         record_error_r1(:,i_record)=error_r1;
                        
                        if Srt(5,5)<=0.01
                            i_singular1=i_singular1+1;
                            record_singular1_degree(:,i_singular1)=[s_tube_fir1;tube_theta_fir1;vertex_m;beta/pi*180;alpha/pi*180;gamma/pi*180];
                            record_singular1_q_c1(:,i_singular1)=q_c1;
                            record_singular1_tube_para1(:,i_singular1)=tube_para1';
                            record_singular1_p_t1(:,i_singular1)=p_t1;
                            record_singular1_R_t1(:,:,i_singular1)=R_t1;
%                             display('Jrt secord order singularity');
                            %%%%%%% draw sentences
                            target_handle=draw_coordinate_system2(1,30,R_t1,p_t1,'rgb','update',target_handle);
                            [denso_cylinerhandle1, denso_cubehandle1, denso_chandle1, denso_lhandle1, denso_ghandle1, denso_coo_handle1, tubehandle1]=draw_denso_dexterity_verify_dai(1,mov1_p_o0,q_c1,tube_para1,'update',denso_cylinerhandle1,denso_cubehandle1,denso_chandle1,denso_lhandle1,denso_ghandle1,denso_coo_handle1,tubehandle1);
                            %%%%%%%
                            saveas(1,['20170723Jrt 2ndorder singular',num2str(i_singular1),'.jpg']);
                        end
                        
                        if Srcm(2,2)<=0.01
                            i_singular2=i_singular2+1;
                            record_singular2_degree(:,i_singular2)=[s_tube_fir1;tube_theta_fir1;vertex_m;beta/pi*180;alpha/pi*180;gamma/pi*180];
                            record_singular2_q_c1(:,i_singular2)=q_c1;
                            record_singular2_tube_para1(:,i_singular2)=tube_para1';
                            record_singular2_p_t1(:,i_singular2)=p_t1;
                            record_singular2_R_t1(:,:,i_singular2)=R_t1;
%                             display('Jrcm secord order singularity');
                            %%%%%%% draw sentences
                            target_handle=draw_coordinate_system2(1,30,R_t1,p_t1,'rgb','update',target_handle);
                            [denso_cylinerhandle1, denso_cubehandle1, denso_chandle1, denso_lhandle1, denso_ghandle1, denso_coo_handle1, tubehandle1]=draw_denso_dexterity_verify_dai(1,mov1_p_o0,q_c1,tube_para1,'update',denso_cylinerhandle1,denso_cubehandle1,denso_chandle1,denso_lhandle1,denso_ghandle1,denso_coo_handle1,tubehandle1);
                            %%%%%%%
                            saveas(1,['20170723Jrcm 2ndorder singular',num2str(i_singular2),'.jpg']);
                        end
                        
                        if Srt(6,6)<=eps
%                             J_rt_plus1 = transpose(J_rt1)/(J_rt1*transpose(J_rt1)+lamda*eye(6));
                            dpl=(1-(Srt(6,6)/eps)^2)*lamda;
%                             J_rt_plus1 = transpose(J_rt1)/(J_rt1*transpose(J_rt1)+dpl*eye(6));
                            J_rt_plus1 = transpose(J_rt1)/(J_rt1*transpose(J_rt1)+0e-5*eye(6)+dpl*Urt(:,6)*Urt(:,6)');
%                             display('denso 1 singularity');
                        else
                            J_rt_plus1=pinv(J_rt1);
                        end
                        q_dot1=pinv(J_rcm_p1)*v_rcm_p1+J_rt_plus1*(x_dot1-J_grip1*pinv(J_rcm_p1)*v_rcm_p1);

%                         record_q_dot1(:,:,i_record)=q_dot1;
%                         r=2.5/24*30;
%                         theta=q_c1(7);
%                         delta=q_c1(8);
%                         J_cq_psi=[-r*cos(delta) r*theta*sin(delta); r*sin(delta) r*theta*cos(delta)];
%                         record_cq_dot(:,i_record)=J_cq_psi*q_dot1(7:8);
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

                     %% set joint limits
                        if (q_c1(1)<=-170/180*pi)
%                             display('denso1_joint_1_lowerboundary')
                            q_c1=q_c1_origin;
                            tube_para1=tube_para1_origin;
                            badcount=badcount_std;
                        end
                        if(q_c1(1)>=170/180*pi)
%                             display('denso1_joint_1_upperboundary')
                            q_c1=q_c1_origin;
                            tube_para1=tube_para1_origin;
                            badcount=badcount_std;
                        end
                        if (q_c1(2)<=-90/180*pi)
%                             display('denso1_joint_2_lowerboundary')
                            q_c1=q_c1_origin;
                            tube_para1=tube_para1_origin;
                            badcount=badcount_std;
                        end
                        if(q_c1(2)>=135/180*pi)
%                             display('denso1_joint_2_upperboundary')
                            q_c1=q_c1_origin;
                            tube_para1=tube_para1_origin;
                            badcount=badcount_std;
                        end
                        if (q_c1(3)<=-80/180*pi)
%                             display('denso1_joint_3_lowerboundary')
                            q_c1=q_c1_origin;
                            tube_para1=tube_para1_origin;
                            badcount=badcount_std;
                        end
                        if(q_c1(3)>=168/180*pi)
%                             display('denso1_joint_3_upperboundary')
                            q_c1=q_c1_origin;
                            tube_para1=tube_para1_origin;
                            badcount=badcount_std;
                        end
                        if (q_c1(4)<=-185/180*pi)
%                             display('denso1_joint_4_lowerboundary')
                            q_c1=q_c1_origin;
                            tube_para1=tube_para1_origin;
                            badcount=badcount_std;
                        end
                        if(q_c1(4)>=185/180*pi)
%                             display('denso1_joint_4_upperboundary')
                            q_c1=q_c1_origin;
                            tube_para1=tube_para1_origin;
                            badcount=badcount_std;
                        end
                        if (q_c1(5)<=-120/180*pi)
%                             display('denso1_joint_5_lowerboundary')
                            q_c1=q_c1_origin;
                            tube_para1=tube_para1_origin;
                            badcount=badcount_std;
                        end
                        if(q_c1(5)>=120/180*pi)
%                             display('denso1_joint_5_upperboundary')
                            q_c1=q_c1_origin;
                            tube_para1=tube_para1_origin;
                            badcount=badcount_std;
                        end
                        if (q_c1(6)<=-2*pi)
%                             display('denso1_joint_6_lowerboundary')
                            q_c1=q_c1_origin;
                            tube_para1=tube_para1_origin;
                            badcount=badcount_std;
                        end
                        if(q_c1(6)>=2*pi)
%                             display('denso1_joint_6_upperboundary')
                            q_c1=q_c1_origin;
                            tube_para1=tube_para1_origin;
                            badcount=badcount_std;
                        end
                        if (q_c1(7)<0)
                            q_c1(7)=-q_c1(7);
                            q_c1(8)=q_c1(8)+pi;
                        end
                        if(q_c1(7)>=2*pi/3)
%                             display('denso1_joint_7_upperboundary')
                            q_c1=q_c1_origin;
                            tube_para1=tube_para1_origin;
                            badcount=badcount_std;
                        end
                       
                     %% cal new RCM position parameter on tube
                        tube_para1=denso_cal_moved_rcmpoint_for_dexerity_verify(mov1_p_o0,q_c1,tube_para1,p_rcm_origin1);
                        
                        [p_c1,R_c1,p_rcm1,R_rcm1,J_grip1,J_denso1,J_rcm_p1]=denso_kinematics_dexterity_verify_dai(mov1_p_o0,q_c1,tube_para1);
                        
                        tube_diff1=p_rcm_origin1-p_rcm1;
                        v_rcm_p1=v_rcm_p_lim*tube_diff1;
                        v_rcm_p1=v_rcm_p1-(v_rcm_p1'*R_rcm1(:,3))*R_rcm1(:,3);
                        if norm(v_rcm_p1) >= v_lim_bound(1)/2
                            v_rcm_p1=v_lim_bound(1)/2*v_rcm_p1/norm(v_rcm_p1);
                        end
                        norm_tube_diff1=norm(tube_diff1);
                        
%                         record_norm_tube_diff(i_record)=norm_tube_diff1;
%                         record_norm_v_rcm_p(i_record)=norm(v_rcm_p1);

                        if norm_tube_diff1>3
%                             display('violate RCM point1')
                            if tube_para1(1)~=s_tube1 || 0
                            i_RCM_violate=i_RCM_violate+1;
                            %%%%%%% draw sentences
                            target_handle=draw_coordinate_system2(1,30,R_t1,p_t1,'rgb','update',target_handle);
                            [denso_cylinerhandle1, denso_cubehandle1, denso_chandle1, denso_lhandle1, denso_ghandle1, denso_coo_handle1, tubehandle1]=draw_denso_dexterity_verify_dai(1,mov1_p_o0,q_c1,tube_para1,'update',denso_cylinerhandle1,denso_cubehandle1,denso_chandle1,denso_lhandle1,denso_ghandle1,denso_coo_handle1,tubehandle1);
                            %%%%%%%
                            saveas(1,['20170728violate RCM',num2str(i_RCM_violate),'.jpg']);
                            record_violate_degree(:,i_RCM_violate)=[s_tube_fir1;tube_theta_fir1;vertex_m;beta/pi*180;alpha/pi*180;gamma/pi*180];
                            record_violate_q_c1(:,i_RCM_violate)=q_c1;
                            record_violate_tube_para1(:,i_RCM_violate)=tube_para1';
                            record_violate_p_t1(:,i_RCM_violate)=p_t1;
                            record_violate_R_t1(:,:,i_RCM_violate)=R_t1;
                            record_violate_q_c_start(:,i_RCM_violate)=q_c_start;
                            record_violate_tube_para_start(:,i_RCM_violate)=tube_para_start';
                            end
                            badcount=badcount_std;
                            q_c1=q_c1_origin;
                            tube_para1=tube_para1_origin;
                            [p_c1,R_c1,p_rcm1,R_rcm1,J_grip1,J_denso1,J_rcm_p1]=denso_kinematics_dexterity_verify_dai(mov1_p_o0,q_c1,tube_para1);
                        end
                        
                        %%%%% draw sentences
%                         if mod(j,300)==0
%                             [denso_cylinerhandle1, denso_cubehandle1, denso_chandle1, denso_lhandle1, denso_ghandle1, denso_coo_handle1, tubehandle1]=draw_denso_dexterity_verify_dai(1,mov1_p_o0,q_c1,tube_para1,'update',denso_cylinerhandle1,denso_cubehandle1,denso_chandle1,denso_lhandle1,denso_ghandle1,denso_coo_handle1,tubehandle1);
%                         end
                        %%%%%
                        
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
                    
%                     %%%%%%% draw sentences
%                     [denso_cylinerhandle1, denso_cubehandle1, denso_chandle1, denso_lhandle1, denso_ghandle1, denso_coo_handle1, tubehandle1]=draw_denso_dexterity_verify_dai(1,mov1_p_o0,q_c1,tube_para1,'update',denso_cylinerhandle1,denso_cubehandle1,denso_chandle1,denso_lhandle1,denso_ghandle1,denso_coo_handle1,tubehandle1);
%                     %%%%%%%
                    
                end
            end
        end
    end
        if s_tube_fir1==0
            break;
        end
    end
end

% %% plot record parameter
% figure
% plot_Srt(1:i_record)=record_Srt(6,6,:);
% plot(1:i_record,plot_Srt)
% title('record Srt')
% % figure
% % plot_Srcm(1:i_record)=record_Srcm(2,2,:);
% % plot(1:i_record,plot_Srcm)
% % title('record Srcm')
% figure
% plot(1:i_record,record_q_dot1(1,1:i_record),'r')
% hold on
% plot(1:i_record,record_q_dot1(2,1:i_record),'b')
% plot(1:i_record,record_q_dot1(3,1:i_record),'c')
% plot(1:i_record,record_q_dot1(4,1:i_record),'y')
% % [~,handle_5,handle_7]=plotyy(1:i_record,record_q_dot1(5,1:i_record),1:i_record,record_cq_dot(1,:));
% % [~,handle_6,handle_8]=plotyy(1:i_record,record_q_dot1(6,1:i_record),1:i_record,record_cq_dot(2,:));
% % set(handle_5,'Color','k');
% % set(handle_6,'Color','g');
% % set(handle_7,'LineStyle','-.');
% % set(handle_8,'LineStyle','--');
% plot(1:i_record,record_q_dot1(5,1:i_record),'k')
% plot(1:i_record,record_q_dot1(6,1:i_record),'g')
% plot(1:i_record,record_cq_dot(1,:),'-.')
% plot(1:i_record,record_cq_dot(2,:),'r-.')
% title('record qdot(1:8)')
% % figure
% % plot(2:i_record,record_q_dot1(8,2:i_record))
% % title('record qdot8')
% figure
% plot(1:i_record,record_norm_tube_diff)
% ylim([0 3]);
% title('record norm tube diff')
% figure
% plot(1:i_record,record_error_p1)
% title('record error p1')
% figure
% plot(1:i_record,record_error_r1)
% title('record error r1')
% figure
% plot(1:i_record,record_v_lim)
% title('record norm v')
% figure
% plot(1:i_record,record_w_lim)
% title('record norm w')
% figure
% plot(1:i_record,record_norm_v_rcm_p)
% title('record norm v rcm p')
% figure
% plot_Sgrip(1:i_record)=record_Sgrip(6,6,:);
% plot(1:i_record,plot_Sgrip)
% title('record Sgrip(6,6)')

%% make movie
% vobj=VideoWriter('testavi10','Uncompressed AVI'); %默认帧率是30fps
% vobj=VideoWriter('two_circle_trocar_20170602','MPEG-4');
% vobj.Quality=100; % set the best video quality
% vobj.FrameRate=10; % FPS
% open(vobj);
% writeVideo(vobj,movie_mov);
% close(vobj);















