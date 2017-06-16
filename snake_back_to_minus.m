%%%%%%%%%%%%%%
%%back to minus
%%%%%%%%%%%%

motorJoint(7)=-10000;
motorJoint(8)=-10000;
motorJoint(9)=0;
fwrite(udp_object,motorJoint(:),'double');