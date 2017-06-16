function udp_obj = UdpSetting()
%此函数UDP的设置函数
%RemoteHost, 远端主机的IPv4地址
%RemotePort, 远端主机的端口号
%LocalPort， 当前主机的端口号
%ByteOrder，当前byteorder是大端还是小端

RemoteHost='192.168.2.200';
RemotePort=8000;

LocalPort=5000;
InputBufferSize=72;

ByteOrder='bigEndian';

udp_obj=udp(RemoteHost,RemotePort,'LocalPort',LocalPort,'InputBufferSize',InputBufferSize,'ByteOrder',ByteOrder);
fopen(udp_obj);
end

