% close all; clear all;
%denso robot resolved rates simulation
%by Jiangran Zhao 20160321

%-------------Init the Omni-------------
% HapticInit();
%---------------------------------------

 %define the initial pose
 q_c=[0;0;0;0;0;0;0;0];
 [p_c, R_c]=cal_pose(q_c);
 
 %initial trocar position
 s_0=400;
 s=s_0;
 [p_trocar R_trocar]=cal_denso_pose(q_c,s);

 
% give a target pose
% p_t=[50,1200,500]';
% R_t=rpy2rot(pi/4,pi/6,pi/8);
%calculate with joint values
% q_t=[pi/18;pi/18;0;0;0;0;pi/4;0];
% [p_t,R_t] = cal_pose(q_t);
% incremental
% p_t=p_c+[50;50;30];
% R_t=R_c*Rpy2Rot(pi/5,pi/10,pi/5);
% R_t=R_c
[p_t,R_t]=OmniMatrix();
% 
% set simulation parameters
eps=0.07;
lamda=0.5;
% lamda=1;
d_t=0.015;
error_p_desire=0.2;
error_r_desire=0.03;
v_lim=200;
w_lim=3;

error_p=norm(p_t-p_c);
R_tc=R_t*R_c';
error_r=acos((R_tc(1,1)+R_tc(2,2)+R_tc(3,3)-1)/2);
if abs(error_r)<=0.01
    r_axis = [0;0;0];
    else
    r_axis = [R_tc(3,2)-R_tc(2,3);R_tc(1,3)-R_tc(3,1);R_tc(2,1)-R_tc(1,2)]/(2*sin(error_r));
end

i=0;
j=0;

[denso_cylinerhandle denso_cubehandle denso_chandle denso_lhandle denso_ghandle denso_coo_handle]=draw_denso(1,q_c,'draw');
%trocar point
plot3(p_trocar(1),p_trocar(2),p_trocar(3),'b*','LineWidth',2);

draw_coordinate_system2(1,30,R_t,p_t,'rgb','draw');

% while (error_p>error_p_desire || error_r>error_r_desire)&&j<3000
while j<10000
   tic
    i=i+1; 
    j=j+1;
    
    %cal x_dot
    v=v_lim.*(p_t - p_c)/norm(p_t - p_c);
    w = w_lim*r_axis;
    x_dot= [v;w];
    
    %calculate J_plus
    J=calculate_sys_J(q_c);
    t=toc
    
      %sigular value robust
  
%     J_plus=pinv(J);
%     %%%%%%
    [J_rcm_p,J_vd]=cal_trocar_jacobian_p(q_c,s);
    
    t3=toc
    
    J_rcm_p_plus=pinv(J_rcm_p);
    
    J_rt=J*(eye(8)-J_rcm_p_plus*J_rcm_p);

    t1=toc
      %sigular value robust
      J_denso=cal_denso_jacobian(q_c);
    [U,S,V] = svd(J_denso);  
%     S1 = S(1:6,1:6);
    if S(6,6)<= eps
%         display(j)
     J_rt_plus = J_rt'/(J_rt*J_rt'+lamda*eye(6));
    else
    J_rt_plus=pinv(J_rt);
    end
%     J_rt_plus=pinv(J_rt);
    
    J_total_plus=(eye(8)-J_rcm_p_plus*J_rcm_p)*J_rt_plus;
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%weight matrix
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  weight=eye(8,8)+[q_c(1)^2 0 0 0 0 0 0 0; 0 q_c(2)^2 0 0 0 0 0 0;0 0 q_c(3)^2 0 0 0 0 0;0 0 0 q_c(4)^2 0 0 0 0;0 0 0 0 q_c(5)^2 0 0 0;0 0 0 0 0 q_c(6)^2 0 0;0 0 0 0 0 0 q_c(7)^2 0;0 0 0 0 0 0 0 q_c(8)^2];
%  weight_inv=inv(weight);
%  
% 
%   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   J_total_plus=weight_inv*(eye(8)-J_rcm_p_plus*J_rcm_p)*J'/(J*weight_inv*(eye(8)-J_rcm_p_plus*J_rcm_p)*J');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% J_total_plus=(eye(8)-J_rcm_p_plus*J_rcm_p)*J'/(J*(eye(8)-J_rcm_p_plus*J_rcm_p)*J');
%%%%%%%%%%%%%%%%%
  
    % %      %sigular value robust
%     [U_r,S_r,V_r]=svd(J_rcm_p);
%     S_r1=S_r(1:6,1:6);
%     if S_r1(6,6)<=2*lamda
%         J_rcm_p_plus=J_rcm_p'*pinv(J_rcm_p*J_rcm_p'+lamda*eye(8));
%     else
%         J_rcm_p_plus=pinv(J_rcm_p);
%     end
%     %%%
%     J_RT=J*(eye(8)-J_rcm_p_plus*J_rcm_p);
%     %%%
%     [U_rt,S_rt,V_rt]=svd(J_RT);
%     S_rt1=S_rt(1:6,1:6);
%     if S_rt1(6,6)<=2*lamda
%        J_RT_plus=J_RT'*pinv(J_RT*J_RT'+lamda*eye(8));
%     else
%         J_RT_plus=pinv(J_RT);
%     end
   
    
    
    
%     
% %      %sigular value robust
%    [U,S,V] = svd(J_t);  
%     S1 = S(1:6,1:6);
%     if S1(5,5)<= 2*lamda
%         J_plus = J_t'*pinv(J_t*J_t'+lamda*eye(6));
% %         J_plus = J'*pinv(J*J'+lamda*eye(6)-lamda*[0;0;0;0;0;1]*[0,0,0,0,0,1]);
%     else
%         J_plus = pinv(J_t);
%     end
% %      J_plus = pinv(J_t);
    %add the increasement to psi.
%     q_dot = J_plus * x_dot;
%     %%to the actuation of backbones
%     %%%%%%%%%%%%%%%
%     %%%%%%%%%%%%%%%
t2=toc
% q_dot=(eye(8)-J_rcm_p_plus*J_rcm_p)*J_RT_plus*x_dot;
q_dot=J_total_plus*x_dot;

%%%%%%%%%%%%%%%%%%%%%%%
%set joint velocity limits
%%%%%%%%%%%%%%%%%%%%%%
denso_q_dot=q_dot(1:6);

% denso_q_dotmax=max(abs(denso_q_dot));
denso_limit=10*[1;1;1;1;1;1];
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
motor_dot=cq_dot*0.1;
motor_limit=20*[1;1];
con_scale=max(abs(motor_dot./motor_limit));
if con_scale>1
    q_dot=q_dot/denso_scale;
end



 %the trocar point on the bar before motion

 [p_former R_former]=cal_denso_pose(q_c,s);
 
    q_c=q_c+q_dot*d_t;
    %%%%%%%%%
    %set joint limits
    if (q_c(7)<0)
        q_c(7)=-q_c(7);
        q_c(8)=q_c(8)+pi;
    end
    if (q_c(1)<=-pi)
        q_c(1)=-pi;
    end
    if(q_c(1)>=pi)
        q_c(1)=pi;
    end
    
% % % % % % % % % % % % % % % % % % % % % % % % %     
    
     [p_c,R_c]=cal_pose(q_c);
     %%%%%%%%%%%%%%%%%%%%%
     %send the joint values to the motors here
     %%%%%%%%%%%%%%%%%%
     
     %the trocar point on the bar after motion
    [p_after R_after]=cal_denso_pose(q_c,s);
     p_mov=p_after-p_former;
     dets=norm(p_mov)*(p_mov'*R_former(:,3)/norm(p_mov'*R_former(:,3)));
     s=s-dets;
     %%%%%%%%%%%%%%%%%%%%%%%%
     %%%%%refresh pt rt
     %%%%%%%%%%%%%%%%
%      tic
     [p_t,R_t]=OmniMatrix();
%      t=toc
     %-------------------read matrix from Omni---------------------------
%      T_Omni=OmniMatrix();
%      R_t=T_Omni(1:3,1:3);
%      p_t=T_Omni(1:3,1);
     %-------------------------------------------------------------------
    error_p=norm(p_t-p_c);
    R_tc=R_t*R_c';
    error_r=acos((R_tc(1,1)+R_tc(2,2)+R_tc(3,3)-1)/2);
    if abs(error_r)<=0.01
          r_axis = [0;0;0];
    else
      r_axis = [R_tc(3,2)-R_tc(2,3);R_tc(1,3)-R_tc(3,1);R_tc(2,1)-R_tc(1,2)]/(2*sin(error_r));
    end
    
    if i == 100000
        q_dot
        error_p
        error_r
        S(6,6)

         [denso_cylinerhandle denso_cubehandle denso_chandle denso_lhandle denso_ghandle denso_coo_handle]=draw_denso(1,q_c,'update',denso_cylinerhandle,denso_cubehandle,denso_chandle,denso_lhandle,denso_ghandle,denso_coo_handle);
%         F = getframe(gcf);
        i=0;
    end
    t4=toc
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







