function [troc_para]=two_denso_cal_moved_troc(mov_p_o0,joint_v,troc_para,p_0_t1former)
% q1=joint_v(1)*180/pi; q2=joint_v(2)*180/pi; q3=joint_v(3)*180/pi; q4=joint_v(4)*180/pi; q5=joint_v(5)*180/pi; q6=joint_v(6)*180/pi;
q1=joint_v(1); q2=joint_v(2); q3=joint_v(3); q4=joint_v(4); q5=joint_v(5); q6=joint_v(6);

%DENSO-VM6083
%d1=475; a1=180; d2=150+96; d3=d2;a2=385; a3=100; d4=445;d6=90;L=40;
%DENSO-VM60B1?
d1=475; a1=180; d2=150+96; d3=d2;a2=520; a3=100; d4=590;d6=90;L=40;

x_axis=[1;0;0]; y_axis=[0;1;0]; z_axis=[0;0;1];
%kinematics

R_0_1=rotz(q1);
p_0_o1=[0;0;d1]+mov_p_o0;

R_1_2=rotx(-pi/2)*rotz(-pi/2+q2);
R_0_2=R_0_1*R_1_2;
p_0_o2=p_0_o1+a1*R_0_1*x_axis+d2*R_0_1*y_axis;

R_2_3=rotz(-pi/2+q3);
R_0_3=R_0_2*R_2_3;
p_0_o3=p_0_o2+a2*R_0_2*x_axis-d3*R_0_2*z_axis;


R_3_4=rotx(-pi/2)*rotz(q4);
R_0_4=R_0_3*R_3_4;
p_0_o4=p_0_o3+a3*R_0_3*x_axis+d4*R_0_4*z_axis;


R_4_5=rotx(pi/2)*rotz(q5);
R_0_5=R_0_4*R_4_5;
p_0_o5=p_0_o4;


R_5_6=rotx(-pi/2)*rotz(q6);
R_0_6=R_0_5*R_5_6;
p_0_o6=p_0_o5+d6*R_0_6*z_axis;

%%%%%%%%%%%%%%%%%%%%%%%%%
% RCM point
switch size(troc_para,2)
    case 3
        s_rcm=troc_para(1); % RCMµã»¡³¤
        s_troc=troc_para(2); % trocar»¡³¤
        troc_r=troc_para(3); % Ô²»¡°ë¾¶
        p_6_o6t1=180*z_axis+[0;troc_r-cos(s_rcm/troc_r)*troc_r;sin(s_rcm/troc_r)*troc_r];
        p_0_t1=p_0_o6+R_0_6*p_6_o6t1;
        %the trocar point after motion
        p_0_t1mov=p_0_t1-p_0_t1former;
        if norm(p_0_t1mov)==0;
            s_rcm_after=s_rcm;
        else
            p_6_rcmafter=R_0_6'*(p_0_t1former-p_0_o6)-180*z_axis;
            s_rcm_after=asin(p_6_rcmafter(3)/troc_r)*troc_r;
        end
        troc_para=[s_rcm_after,s_troc,troc_r];
    case 4
        s_rcm=troc_para(1); % RCMµã»¡³¤
        s_troc=troc_para(2); % trocar»¡³¤
        troc_r_fir=troc_para(3); % µÚÒ»¶ÎÔ²»¡°ë¾¶
        troc_r_sec=troc_para(4); % µÚ¶þ¶ÎÔ²»¡°ë¾¶
        if s_rcm<s_troc/2
            p_6_o6t1=180*z_axis+[0;troc_r_fir-cos(s_rcm/troc_r_fir)*troc_r_fir;sin(s_rcm/troc_r_fir)*troc_r_fir];
            p_0_t1=p_0_o6+R_0_6*p_6_o6t1;
            %the trocar point after motion
            p_0_t1mov=p_0_t1-p_0_t1former;
            if norm(p_0_t1mov)==0;
                s_rcm_after=s_rcm;
            else
                p_6_rcmafter=R_0_6'*(p_0_t1former-p_0_o6)-180*z_axis;
                s_rcm_after_fir=asin(p_6_rcmafter(3)/troc_r_fir)*troc_r_fir;
                if s_rcm_after_fir>s_troc/2
                    p_cir1_cir1rcm=(rotx(-s_troc/2/troc_r_fir))'*(p_6_rcmafter-[0;troc_r_fir-cos(s_troc/2/troc_r_fir)*troc_r_fir;sin(s_troc/2/troc_r_fir)*troc_r_fir]);
                    s_rcm_after_sec=asin(p_cir1_cir1rcm(3)/troc_r_sec)*troc_r_sec;
                    s_rcm_after=s_rcm_after_fir+s_rcm_after_sec;
                else
                    s_rcm_after=s_rcm_after_fir;
                end
            end
        else
            p_6_o6t1=180*z_axis+[0;troc_r_fir-cos(s_troc/2/troc_r_fir)*troc_r_fir;sin(s_troc/2/troc_r_fir)*troc_r_fir]+rotx(-s_troc/2/troc_r_fir)*[0;troc_r_sec-cos((s_rcm-s_troc/2)/troc_r_sec)*troc_r_sec;sin((s_rcm-s_troc/2)/troc_r_sec)*troc_r_sec];
            p_0_t1=p_0_o6+R_0_6*p_6_o6t1;
            %the trocar point after motion
            p_0_t1mov=p_0_t1-p_0_t1former;
            if norm(p_0_t1mov)==0;
                s_rcm_after=s_rcm;
            else
                p_6_rcmafter=R_0_6'*(p_0_t1former-p_0_o6)-180*z_axis;
                s_rcm_after_fir=asin(p_6_rcmafter(3)/troc_r_fir)*troc_r_fir;
                if s_rcm_after_fir>s_troc/2
                    p_cir1_cir1rcm=(rotx(-s_troc/2/troc_r_fir))'*(p_6_rcmafter-[0;troc_r_fir-cos(s_troc/2/troc_r_fir)*troc_r_fir;sin(s_troc/2/troc_r_fir)*troc_r_fir]);
                    s_rcm_after_sec=asin(p_cir1_cir1rcm(3)/troc_r_sec)*troc_r_sec;
                    s_rcm_after=s_troc/2+s_rcm_after_sec;
                else
                    s_rcm_after=s_rcm_after_fir;
                end
            end
        end
        troc_para=[s_rcm_after,s_troc,troc_r_fir,troc_r_sec];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end
