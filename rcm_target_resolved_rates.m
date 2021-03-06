function [q_c1,tube_para1]=rcm_target_resolved_rates(q_c1,mov1_p_o0,p_0_rcm,R_0_rcm,tube_para1)
[p_c1,R_c1,J_tube]=rcm_denso_kinematics(mov1_p_o0,q_c1,tube_para1);
p_t1=p_0_rcm;
R_t1=R_0_rcm;

% set simulation parameters
eps=0.07;
lamda=4;
d_t=0.002;
error_p_desire=0.01;
error_r_desire=0.03;
v_lim=15;
w_lim=0.6;
v_lim_low=0.5;
w_lim_low=0.05;
steplimit=8000000;

%%%%%%%%%%%%%

% [denso_cylinerhandle1, denso_cubehandle1, denso_chandle1, denso_lhandle1, denso_ghandle1, denso_coo_handle1,tubehandle1]=draw_denso_dexterity_verify_dai(1,mov1_p_o0,q_c1,tube_para1,'draw');

%tube point
% plot3(p_0_rcm(1),p_0_rcm(2),p_0_rcm(3),'b*','LineWidth',3);
% draw_coordinate_system2(1,60,R_t1,p_t1,'rgb','draw');
j=0;

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
        tube_para1(1)=tube_para1(2)+tube_para1(3);
%         [denso_cylinerhandle1, denso_cubehandle1, denso_chandle1, denso_lhandle1, denso_ghandle1, denso_coo_handle1, tubehandle1]=draw_denso_dexterity_verify_dai(1,mov1_p_o0,q_c1,tube_para1,'update',denso_cylinerhandle1,denso_cubehandle1,denso_chandle1,denso_lhandle1,denso_ghandle1,denso_coo_handle1,tubehandle1);
        display('denso tube initialization successful');
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
    
    %sigular value robust
    [U1,S1,V1] = svd(J_tube);
    if S1(6,6)<= eps % paper中是0.02
        J_tube_plus1 = transpose(J_tube)/(J_tube*transpose(J_tube)+lamda*eye(6));
        display('denso tube initialization singularity');
    else
        J_tube_plus1=pinv(J_tube);
    end
    
    q_dot1=J_tube_plus1*x_dot1;
    
    q_c1(1:6)=q_c1(1:6)+q_dot1*d_t;
    
    
    %%%%%%%%%
    %set joint limits
    if (q_c1(1)<=-pi*5/6)
        display('denso1_joint_1_lowerboundary_in_tube_initialization')
        q_c1(1)=-pi*5/6;
    end
    if(q_c1(1)>=pi*5/6)
        display('denso1_joint_1_upperboundary_in_tube_initialization')
        q_c1(1)=pi*5/6;
    end
    if (q_c1(2)<=-pi/3)
        display('denso1_joint_2_lowerboundary_in_tube_initialization')
        q_c1(2)=-pi/3;
    end
    if(q_c1(2)>=3*pi/4)
        display('denso1_joint_2_upperboundary_in_tube_initialization')
        q_c1(2)=3*pi/4;
    end
    if (q_c1(3)<=-4*pi/9)
        display('denso1_joint_3_lowerboundary_in_tube_initialization')
        q_c1(3)=-4*pi/9;
    end
    if(q_c1(3)>=11*pi/12)
        display('denso1_joint_3_upperboundary_in_tube_initialization')
        q_c1(3)=11*pi/12;
    end
    if (q_c1(4)<=-pi)
        display('denso1_joint_4_lowerboundary_in_tube_initialization')
        q_c1(4)=-pi;
    end
    if(q_c1(4)>=pi)
        display('denso1_joint_4_upperboundary_in_tube_initialization')
        q_c1(4)=pi;
    end
    if (q_c1(5)<=-2*pi/3)
        display('denso1_joint_5_lowerboundary_in_tube_initialization')
        q_c1(5)=-2*pi/3;
    end
    if(q_c1(5)>=2*pi/3)
        display('denso1_joint_5_upperboundary_in_tube_initialization')
        q_c1(5)=2*pi/3;
    end
    if (q_c1(6)<=-2*pi)
        display('denso1_joint_6_lowerboundary_in_tube_initialization')
        q_c1(6)=-2*pi;
    end
    if(q_c1(6)>=2*pi)
        display('denso1_joint_6_upperboundary_in_tube_initialization')
        q_c1(6)=2*pi;
    end
    % % % % % % % % % % % % % % % % % % % % % % % % %
    
    [p_c1,R_c1,J_tube]=rcm_denso_kinematics(mov1_p_o0,q_c1,tube_para1);
    
%     if mod(j,300)==0
%     [denso_cylinerhandle1, denso_cubehandle1, denso_chandle1, denso_lhandle1, denso_ghandle1, denso_coo_handle1, tubehandle1]=draw_denso_dexterity_verify_dai(1,mov1_p_o0,q_c1,tube_para1,'update',denso_cylinerhandle1,denso_cubehandle1,denso_chandle1,denso_lhandle1,denso_ghandle1,denso_coo_handle1,tubehandle1);
%     end
end
if j==steplimit
error('can not reach rcm point');
end
end