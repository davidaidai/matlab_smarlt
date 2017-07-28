function v=findV(i,Cd,eps)
Cd_inip=Cd(:,i:size(Cd,2))-Cd(:,i)*ones(1,size(Cd(:,i:size(Cd,2)),2)); % 原点align到第i个点
thetai=atan2(Cd(2,i+1)-Cd(2,i),Cd(3,i+1)-Cd(3,i));
Cd_ini=rotx(thetai)*Cd_inip; % z轴指向align到第i和i+1点连线形成的指向
m=1;
while 1
    d(m)=norm(Cd_ini(:,m+1)-Cd_ini(:,1));
    if d(m)<=eps
        m=m+1;
    else
        break;
    end
end
phi(m)=atan2(Cd_ini(2,m+1)-Cd_ini(2,1),Cd_ini(3,m+1)-Cd_ini(3,1));
psai(m)=asin(eps/sqrt((Cd_ini(2,m+1)-Cd_ini(2,1))^2+(Cd_ini(3,m+1)-Cd_ini(3,1))^2));
S(1:2,m)=[phi(m)-psai(m);phi(m)+psai(m)];
T(1:2,m)=S(1:2,m);
while 1
    m=m+1;
    phi(m)=atan2(Cd_ini(2,m+1)-Cd_ini(2,1),Cd_ini(3,m+1)-Cd_ini(3,1));
    psai(m)=asin(eps/sqrt((Cd_ini(2,m+1)-Cd_ini(2,1))^2+(Cd_ini(3,m+1)-Cd_ini(3,1))^2));
    d(m)=norm(Cd_ini(:,m+1)-Cd_ini(:,1));
    S(1:2,m)=[phi(m)-psai(m);phi(m)+psai(m)];
    T(1:2,m)=[max([T(1,m-1),S(1,m)]);min([T(2,m-1),S(2,m)])];
    if T(1,m)>T(2,m)
        while 1
            m=m-1;
            if phi(m)>=T(1,m) && phi(m)<=T(2,m)
                break;
            end
        end
        break;
    end
    if m==size(Cd,2)-i
        break
    end
       
end
while 1
    sign=0;
    if d(m)>=max(d(1:m-1))
        break;
    else
        diff_d=max(d(1:m-1)-d(m));
        indice=find(diff_d>0);
        for j=1:size(indice)
        err_pt2ln=cal_err_pt2ln(Cd_ini(:,1),Cd_ini(:,m),Cd_ini(:,indice(j)));
        if err_pt2ln>eps
            m=m-1;
            sign=1;
            break;
        end
        end
        if sign==0
            break;
        end
    end
end
v=m+i;
end
        
