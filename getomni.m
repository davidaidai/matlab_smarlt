function [omni_data,button] = getomni()
% �˺����Ƿ���Omni����4x4�����ֵ

if not(libisloaded('PhantomA'))
    HapticInit();
end
data1=zeros(1,17);
% data2=zeros(1,16);
DataPtr1=libpointer('singlePtr',data1);
% DataPtr2=libpointer('singlePtr',data2);
calllib('PhantomA','GetHapticDataA',DataPtr1);
% calllib('Phantom','GetHapticDataB',DataPtr2);
omni_data=DataPtr1.Value(1:16);
button=DataPtr1.Value(17);
end

