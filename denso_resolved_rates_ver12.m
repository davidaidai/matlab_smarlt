%simiplified denso_continuum control
%by Jiangran Zhao 20160420

%define the initial pose
q_ini=[0;0;pi*2/3;0;0;0;0;0];
griper_c=0;
q_c=q_ini;
%define the trocar position
s_0=400;
s=s_0;
[p_c,R_c,p_troc,R_troc,J_grip,J_denso,J_rcm_p]=denso_kinematic(q_c,s);
p_trocar=p_troc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% p_t=p_c+[-50;50;30];
% R_t=R_c*Rpy2Rot(pi/5,pi/10,pi/15);
% % 
% [omni_data,button]=getomni();
% Omni_command=ones(5,1)*omni_data;
% [p_t,R_t,button]=cal_target(Omni_command,button);

[p_t,R_t,button]=OmniMatrix();


% set simulation parameters
eps=0.07;
lamda=0.5;
d_t=0.002;
error_p_desire=0.2;
error_r_desire=0.03;
v_lim=15;
w_lim=0.4;
%%%%%%%%%%%%%
error_p=norm(p_t-p_c);
R_tc=R_t*R_c';
error_r=acos((R_tc(1,1)+R_tc(2,2)+R_tc(3,3)-1)/2);
if abs(error_r)<=0.01
    r_axis = [0;0;0];
    else
    r_axis = [R_tc(3,2)-R_tc(2,3);R_tc(1,3)-R_tc(3,1);R_tc(2,1)-R_tc(1,2)]/(2*sin(error_r));
end

i=0;j=0;k=1;
[denso_cylinerhandle denso_cubehandle denso_chandle denso_lhandle denso_ghandle denso_coo_handle]=draw_denso(1,q_c,'draw');
%trocar point
plot3(p_trocar(1),p_trocar(2),p_trocar(3),'b*','LineWidth',2);
draw_coordinate_system2(1,30,R_t,p_t,'rgb','draw');
steplimit=800000000;

motor1_stat=0;
motor2_stat=0;

while j<steplimit
%     (error_p>error_p_desire || error_r>error_r_desire)&&j<30000
%     tic;
    i=i+1; 
    j=j+1;
%     if mod(j,2)==0
%         disp(j);
%         disp(motorJoint);
%     end
    
    %cal x_dot
    v=v_lim.*(p_t - p_c)/norm(p_t - p_c);
    w = w_lim*r_axis;
    x_dot= [v;w];
    
    J_rcm_p_plus=pinv(J_rcm_p);
    J_rt=J_grip*(eye(8)-J_rcm_p_plus*J_rcm_p);
     %sigular value robust
    [U,S,V] = svd(J_denso);  
    if S(6,6)<= eps
     J_rt_plus = transpose(J_rt)/(J_rt*transpose(J_rt)+lamda*eye(6));
    else
    J_rt_plus=pinv(J_rt);
    end
    J_total_plus=(eye(8)-J_rcm_p_plus*J_rcm_p)*J_rt_plus;

    q_dot=J_total_plus*x_dot;

%%%%%%%%%%%%%%%%%%%%%%%
%set joint velocity limits
%%%%%%%%%%%%%%%%%%%%%%
    denso_q_dot=q_dot(1:6);
    denso_limit=0.3*[1;1;1;1;1;1];
    denso_scale=max(abs(denso_q_dot./denso_limit));
    if denso_scale>1
        q_dot=q_dot/denso_scale;
    end
    con_psi_dot=q_dot(7:8);
    r=2.5/48*60;
    theta=q_c(7);
    delta=q_c(8);
    theta_dot=q_dot(7);
    delta_dot=q_dot(8);
    J_cq_psi=[-r*cos(delta) r*theta*sin(delta); r*sin(delta) r*theta*cos(delta)];
    cq_dot=J_cq_psi*con_psi_dot;
    motor_dot=cq_dot*8.83;
    motor_limit=500/6*[1;1];
    con_scale=max(abs(motor_dot./motor_limit));
    if con_scale>1
        q_dot=q_dot/denso_scale;
    end

 %the trocar point on the bar before motion
    p_former=p_troc; R_former=R_troc;
    q_c=q_c+q_dot*d_t;
    %%%%%%%%%
    %set joint limits
    if (q_c(7)<0)
        q_c(7)=-q_c(7);
        q_c(8)=q_c(8)+pi;
    end
    if (q_c(1)<=-pi*5/6)
        display('joint_1_lowerboundary')
        j=steplimit;
    end
    if(q_c(1)>=pi*5/6)
        display('joint_1_upperboundary')
        j=steplimit;
    end
     if (q_c(2)<=-pi/3)
        display('joint_2_lowerboundary')
        j=steplimit;
    end
    if(q_c(2)>=3*pi/4)
        display('joint_2_upperboundary')
        j=steplimit;
    end
     if (q_c(3)<=-4*pi/9)
        q_c(3)=-pi;
        display('joint_3_lowerboundary')
        j=steplimit;
    end
    if(q_c(3)>=11*pi/12)
        q_c(3)=pi;
        display('joint_3_upperboundary')
        j=steplimit;
    end
     if (q_c(4)<=-pi)
        q_c(4)=-pi;
        display('joint_4_lowerboundary')
        j=steplimit;
    end
    if(q_c(4)>=pi)
        q_c(4)=pi;
        display('joint_4_upperboundary')
        j=steplimit;
    end
     if (q_c(5)<=-2*pi/3)
        q_c(5)=-pi;
        display('joint_5_lowerboundary')
        j=steplimit;
    end
    if(q_c(5)>=2*pi/3)
        q_c(5)=pi;
        display('joint_5_upperboundary')
        j=steplimit;
    end
    if (q_c(6)<=-2*pi)
        q_c(6)=-pi;
        display('joint_6_lowerboundary')
        j=steplimit;
    end
    if(q_c(6)>=2*pi)
        q_c(6)=pi;
        display('joint_6_upperboundary')
        j=steplimit;
    end
    if(q_c(7)>=pi/1.5)
        q_c(7)=pi/1.5;
        display('joint_7_upperboundary')
        j=steplimit;
    end
  
% % % % % % % % % % % % % % % % % % % % % % % % %     
    
     %%%%%%%%%%%%%%%%%%%%%
     %send the joint values to the motors here
      motorJoint=convert2motorJoints(q_c,griper_c);
      
     motor1_tc=motorJoint(7);
     motor2_tc=motorJoint(8);
     k1=1.246;
     motor1_com=k1*motor1_tc;
%      motor1_dir=motor1_com-motor1_stat;
%      motor1_stat=motor1_com;
%      if motor1_dir>=0
%          motor1_index=0;
%      else
%          motor1_index=1;
%      end
%      backlash1=8800;
%      motor1_given=motor1_com-motor1_index*backlash1;
     motor1_given=motor1_com;
     
     k2=1.24;
     motor2_com=k2*motor2_tc;
%      motor2_dir=motor2_com-motor2_stat;
%      motor2_stat=motor2_com;
%      if motor2_dir>=0
%          motor2_index=0;
%      else
%          motor2_index=1;
%      end
%      backlash2=9800;
%      motor2_given=motor2_com-motor2_index*backlash2;
     motor2_given=motor2_com;
      
     motorJoint(7)=motor1_given;
     motorJoint(8)=motor2_given;
      
%       fwrite(udp_object,motorJoint(:),'double');
     
     %%%%%%%%%%%%%%%%%%
    [p_after R_after]=cal_moved_troc(q_c,s);
     %the trocar point on the bar after motion
     p_mov=p_after-p_former;
     if norm(p_mov)==0;
         dets=0;
     else
         dets=norm(p_mov)*(p_mov'*R_former(:,3)/norm(p_mov'*R_former(:,3)));
     end
     s=s-dets;
      [p_c,R_c,p_troc,R_troc,J_grip,J_denso,J_rcm_p]=denso_kinematic(q_c,s);
      troc_diff=p_trocar-p_troc;
      norm_troc_diff=norm(troc_diff);
      if norm_troc_diff>1
          display('vialate RCM point')
          display(j)
          j=steplimit;
      end
     %%%%%%%%%%%%%%%%%%%%%%%%
     %%%%%refresh pt rt
%      [Omni_command(k,:),button]=getomni();
%      k=k+1;
%      if k==5
%          k=1;
%      end
%      [p_t,R_t,button]=cal_target(Omni_command,button); 
     
     [p_t,R_t,button]=OmniMatrix();
     griper_c=griper_c+10*button;
     if griper_c>=50000
         griper_c=50000;
     else if griper_c<=0
             griper_c=0;
         end
     end

     %%%%%%%%%%%%%%%%
    error_p=norm(p_t-p_c);
    R_tc=R_t*R_c';
    error_r=acos((R_tc(1,1)+R_tc(2,2)+R_tc(3,3)-1)/2);
    
    if abs(error_r)<=0.01
          r_axis = [0;0;0];
    else
      r_axis = [R_tc(3,2)-R_tc(2,3);R_tc(1,3)-R_tc(3,1);R_tc(2,1)-R_tc(1,2)]/(2*sin(error_r));
    end
%    toc;
%    t_6=toc
%    disp(t_6);
    if i == 2000
        q_dot
%         motorJoint
        error_p
        error_r
         [denso_cylinerhandle denso_cubehandle denso_chandle denso_lhandle denso_ghandle denso_coo_handle]=draw_denso(1,q_c,'update',denso_cylinerhandle,denso_cubehandle,denso_chandle,denso_lhandle,denso_ghandle,denso_coo_handle);
        F = getframe(gcf);
%          axis([-500 500 1000 1400 0 800]);
%          view(180,0)
        i=0;
    end
%  if mod(j,50)==0
%       saveas(1,['resolve',num2str(j),'.jpg']);
%  end
%    
end
   q_dot
   error_p
   error_r
j
[denso_cylinerhandle denso_cubehandle denso_chandle denso_lhandle denso_ghandle denso_coo_handle]=draw_denso(1,q_c,'update',denso_cylinerhandle,denso_cubehandle,denso_chandle,denso_lhandle,denso_ghandle,denso_coo_handle);



















