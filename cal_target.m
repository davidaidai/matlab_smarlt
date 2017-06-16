function [pt,R_t,Button] = cal_target(Omni_command,Button)
% 此函数是返回Omni矩阵4x4矩阵的值


MatrixA=Omni_command;
% Button=DataPtr1.Value(17);
if Button==2
    Button=-1;
elseif Button==3
    Button=0;
end
% MatrixB=reshape(MatrixB,4,4);
init_P_Omni=[2.329;5.4;33.9];
scale=1;
T_0_1=MatrixA;
T_0_1(1:3,4)=(T_0_1(1:3,4)-init_P_Omni)/scale;
R_m_0=rotd([1;0;0],pi/2);
R_1_2=rotd([1;0;0],pi)*rotd([0;0;1],-pi/2);
T_m_0=eye(4);
T_m_0(1:3,1:3)=R_m_0;
T_1_2=eye(4);
T_1_2(1:3,1:3)=R_1_2;

init_P_tip=[-177.1;1037.8;187.1];

T_m_2=T_m_0*T_0_1*T_1_2;
pt=T_m_2(1:3,4)+init_P_tip;
% R_t=T_m_2(1:3,1:3);
R_t=rotx(-pi/9)*T_m_2(1:3,1:3);

end

