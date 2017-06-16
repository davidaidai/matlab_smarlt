%%设置一个udp_get的object，分别是发送端机器的IP，端口，本地机器的端口，每次传输的数据大小（16个数据128byte），以及byteorder（针对windows
%%viusal studio，选择little ndian）
udp_get=udp('192.168.2.20',10000,'LocalPort',22222,'InputBufferSize',128,'ByteOrder','littleEndian' );
fopen(udp_get);
%%设置omni的转换坐标系
T_m1_0=Rot('x',pi);
T_m1_0=[T_m1_0,zeros(3,1);zeros(1,3),1];
T_1_2=Rot('x',pi);
T_1_2=[T_1_2,zeros(3,1);zeros(1,3),1];
%%进入循环，读取udp
while 1
    [A,count,msg] = fread(udp_get,32,'double');
    [m n]=size(A);
    if m==16 % in case no udp is received
       B=A(17:32);
       T_0_1(1:3,1:3)=[B(1:3),B(5:7),B(9:11)];
       %change scale and set origin point of snake endeffector to
       %[0,0,60]，the initial point of omni device is [0;0;15]
       T_0_1(1:3,4)=(B(13:15)-[0;0;15])/scale-[0;0;60];
       T_m1_2=T_m1_0*T_0_1*T_1_2;

    end
   %use T_m1_2 below
end