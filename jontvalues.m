figure(3)
xaxis=1:200000;
plot(xaxis,MJ(1:200000,1),'r');
hold on
plot(xaxis,MJ(1:200000,2),'g');
plot(xaxis,MJ(1:200000,3),'b');
plot(xaxis,MJ(1:200000,4),'k');
plot(xaxis,MJ(1:200000,5),'c');
plot(xaxis,MJ(1:200000,6),'m');
figure(4)
plot(xaxis,MJ(:,7),'r');
hold on
plot(xaxis,MJ(:,8),'b');
figure(5)
plot(xaxis,theta_print,'b');
hold on
plot(xaxis,delta_print,'r');

theta_dif=diff(MJ(:,7));
figure(6)
xaaa=1:199999;
plot(xaaa,theta_dif,'b');




