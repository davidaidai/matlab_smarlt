%%%%
%omni test fig
figure(4)
l=1;
omni_unfilt=1:1000;
plot(omni_unfilt,unfilt_ang_axis(:,l),'r');
hold on
omni_filt5=1:990;
plot(omni_filt5,filt5_ang_axis(:,l),'b');
omni_filt50=25:974;
plot(omni_filt50,filt50_ang_axis(:,l),'g');
