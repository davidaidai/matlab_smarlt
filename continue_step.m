n=1;
while n<=106

% p_t=p_c+[5;0;-3];
% R_t=roty(-pi/10005)*R_c;
p_t=pt(1:3,n);
R_t=Rt(1:3,n:n+2);
l=0;
movestep=4000;
while l<movestep
%     (error_p>error_p_desire || error_r>error_r_desire)&&j<30000
%     tic;
    i=i+1; 
    j=j+1;
    l=l+1;

    
    %cal x_dot
    if error_p<=0.5 && error_r<=0.05
         v=v_lim_low.*(p_t - p_c)/norm(p_t - p_c);
         w = w_lim_low*r_axis;
    else
         v=v_lim.*(p_t - p_c)/norm(p_t - p_c);
         w = w_lim*r_axis;
    end
    x_dot= [v;w];
    %%%%%%%%%%%%%%%%%%%%%%%%
        J_rcm_p_plus=pinv(J_rcm_p);

 %%%%%%%%%%%%%%%%%%%%%%%%%%% 
 
    J_rt=J_grip*(eye(8)-J_rcm_p_plus*J_rcm_p);
     %sigular value robust
    [Urt,Srt,Vrt] = svd(J_rt);  
    if Srt(5,5)<= eps
     J_rt_plus = transpose(J_rt)/(J_rt*transpose(J_rt)+lamda*eye(6));
%      disp(j);
%      disp('rt singular')
%      q_c
    else
    J_rt_plus=pinv(J_rt);
%      J_rt=eye(8,8)/Weight*transpose(J_rt)/(J_rt/Weight*transpose(J_rt));
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
     motor1_given=round(motor1_com);
     
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
     motor2_given=round(motor2_com);
      
     motorJoint(7)=motor1_given;
     motorJoint(8)=motor2_given;
     
%      MJ(j,:)=motorJoint;
      fwrite(udp_object,motorJoint(:),'double');
     
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
      if norm_troc_diff>3
          display('vialate RCM point')
          display(j)
          j=steplimit;
      end
     %%%%%%%%%%%%%%%%%%%%%%%%
     %%%%%refresh pt rt

     
%      [p_t,R_t,button]=OmniMatrix();
     griper_c=griper_c-3*button;
     if griper_c>=0
         griper_c=0;
     else if griper_c<=-4000
             griper_c=-4000;
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
% 
%     if i == 2000
%         q_dot
% %         motorJoint
%         error_p
%         griper_c
%         error_r
%         j
% %          [denso_cylinerhandle denso_cubehandle denso_chandle denso_lhandle denso_ghandle denso_coo_handle]=draw_denso(1,q_c,'update',denso_cylinerhandle,denso_cubehandle,denso_chandle,denso_lhandle,denso_ghandle,denso_coo_handle);
% %         F = getframe(gcf);
% % %          axis([-500 500 1000 1400 -200 600]);
% % %          view(180,0)
% %         i=0;
%     end
% %  if mod(j,50)==0
% %       saveas(1,['resolve',num2str(j),'.jpg']);
% %  end
% %    
end
n=n+1;
end
