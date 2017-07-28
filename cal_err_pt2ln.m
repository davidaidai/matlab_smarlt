function err=cal_err_pt2ln(p1,p2,p3)
    p=[p1,p2,p3];
    p_inip=p-p1;
    theta=atan2(p2(2)-p1(2),p2(3)-p1(3));
    p_ini=rotx(theta)*p_inip;
    if p_ini(3,3)<0
        err=norm(p_ini(:,3)-p_ini(:,1));
    elseif p_ini(3,3)>p_ini(3,2)
        err=norm(p_ini(:,3)-p_ini(:,2));
    else
        err=abs(p_ini(2,3));
    end
end