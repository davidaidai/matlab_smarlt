function udp_obj = UdpSetting()
%�˺���UDP�����ú���
%RemoteHost, Զ��������IPv4��ַ
%RemotePort, Զ�������Ķ˿ں�
%LocalPort�� ��ǰ�����Ķ˿ں�
%ByteOrder����ǰbyteorder�Ǵ�˻���С��

RemoteHost='192.168.2.200';
RemotePort=8000;

LocalPort=5000;
InputBufferSize=72;

ByteOrder='bigEndian';

udp_obj=udp(RemoteHost,RemotePort,'LocalPort',LocalPort,'InputBufferSize',InputBufferSize,'ByteOrder',ByteOrder);
fopen(udp_obj);
end

