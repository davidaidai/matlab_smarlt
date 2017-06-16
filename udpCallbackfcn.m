function udpCallbackfcn( obj,event)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

if obj.BytesAvailable<5
    if 's'==fread(obj,1,'char')
        obj.UserData(1)=1;
    else
        obj.UserData(1)=0;
    end
else
    a=zeros(9,1);
    a(1:9,1)=fread(obj,9,'double');
    obj.UserData(1,2:10)=a(:);
end

end

