if not(libisloaded('PhantomA'))
    HapticInit();
end
data1=zeros(1,17);
% data2=zeros(1,16);
DataPtr1=libpointer('singlePtr',data1);
% DataPtr2=libpointer('singlePtr',data2);
calllib('PhantomA','GetHapticDataA',DataPtr1);
% calllib('Phantom','GetHapticDataB',DataPtr2);
MatrixA=DataPtr1.Value(1:16);
% MatrixB=DataPtr2.Value;
MatrixA=reshape(MatrixA,4,4);
Button=DataPtr1.Value(17);
if Button==2
    Button=-1;
elseif Button==3
    Button=0;
end
% MatrixB=reshape(MatrixB,4,4);
init_P_Omni=[0;-89;-57];
scale=1;
T_0_1=MatrixA;
T_0_1(1:3,4)=(T_0_1(1:3,4)-init_P_Omni)/scale;
R_m_0=rotd([1;0;0],pi/2);
R_1_2=rotd([1;0;0],pi)*rotd([0;0;1],-pi/2);
T_m_0=eye(4);
T_m_0(1:3,1:3)=R_m_0;
T_1_2=eye(4);
T_1_2(1:3,1:3)=R_1_2;

init_P_tip=[0;1350;960];

T_m_2=T_m_0*T_0_1*T_1_2;
pt=T_m_2(1:3,4)+init_P_tip;
R_t=T_m_2(1:3,1:3);

draw_coordinate_system2(1,30,R_t,p_t,'rgb','draw');
    axis equal;
    axis([-500 500 -200 1500 0 1200]);
    grid on 
    view(60,30)