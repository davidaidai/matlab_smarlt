function v=findV(i,Cd,eps)
Cd_inip=Cd-Cd(:,i)*ones(1,size(Cd,2));
thetai=atan2(Cd(2,i+1)-Cd(2,i),Cd(3,i+1)-Cd(3,i));
Cd_ini=rotx(thetai)*Cd_inip;
end