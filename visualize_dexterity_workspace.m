%                         tube_para1=denso_cal_moved_rcmpoint_for_dexerity_verify(mov1_p_o0,q_c1,tube_para1,p_rcm_origin1);

    figure
    axis equal
    axis tight
    view(60,30)
    xlabel('x(mm)')
    ylabel('y(mm)')
    zlabel('z(mm)')
    hold on
    %%
r=60;
coloum=2;
size=0;
for i=1:max(find(record_gamma_degree(3,:,coloum)))
    R_t=roty(record_gamma_degree(4,i,coloum)/180*pi)*rotx(record_gamma_degree(5,i,coloum)/180*pi)*rotz(record_gamma_degree(6,i,coloum)/180*pi);
    p_t=[0;0;r];
    p=R_t*p_t;
    if i~=1
    if record_gamma_degree(4,i-1,coloum)/180*pi==record_gamma_degree(4,i,coloum)/180*pi && record_gamma_degree(5,i-1,coloum)/180*pi==record_gamma_degree(5,i,coloum)/180*pi
        size=size+4;
    else
        size=0;
    end
    end
    plot3(p(1),p(2),p(3),'ro','MarkerSize',4+size)
%     draw_coordinate_system_only_for_zaxis(1,r,R_t,p_t,'rgb','draw');
end
%% draw sphere skelton
r=60;
for beta=0:15/180*pi:(pi-15/180*pi)
    for alpha=0:15/180*pi:(2*pi-15/180*pi)
        Rs=roty(beta)*rotx(alpha);
        ps=[0;0;r];
        p=Rs*ps;
        plot3(p(1),p(2),p(3),'b.');
        hold on;
    end
end
% for phi=0:15/180*pi:(2*pi-15/180*pi)
%     for theta=0:15/180*pi:(2*pi-15/180*pi)
%         x=r*cos(theta)*sin(phi);
%         y=r*cos(theta)*cos(phi);
%         z=r*sin(theta);
%         plot3(x,y,z,'r*');
%     end
% end

%%
for mm=1:1
    jj=0;
    for ii=1:size(record_gamma_degree(:,:,mm),2)-1
        if (record_gamma_degree(5,ii,mm)~=record_gamma_degree(5,ii+1,mm))||(record_gamma_degree(4,ii,mm)~=record_gamma_degree(4,ii+1,mm) && record_gamma_degree(5,ii,mm)==record_gamma_degree(5,ii+1,mm))
            jj=jj+1;
            beta_alpha(1:2,jj,mm)=record_gamma_degree(4:5,ii,mm);
        end
    end
    num=max(find(record_gamma_degree(3,:,mm)));
    fprintf([num2str(num),'\n',num2str(jj),'\n','\n']);
end