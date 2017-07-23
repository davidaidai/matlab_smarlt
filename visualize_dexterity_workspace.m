%                         tube_para1=denso_cal_moved_rcmpoint_for_dexerity_verify(mov1_p_o0,q_c1,tube_para1,p_rcm_origin1);

%     figure
%     axis equal
%     axis tight
%     view(60,30)
%     xlabel('x(mm)')
%     ylabel('y(mm)')
%     zlabel('z(mm)')
for i=1:483
    R_t=rotx(record_gamma_degree(5,i,6))*roty(record_gamma_degree(4,i,6))*rotz(record_gamma_degree(6,i,6));
    p_t=[0;0;30];
    draw_coordinate_system_only_for_zaxis(1,60,R_t,p_t,'rgb','draw');
end

%%
jj=1;
for ii=1:185
    if record_gamma_degree(5,ii,1)~=record_gamma_degree(5,ii+1,1)
    beta_alpha(1:2,jj)=record_gamma_degree(4:5,ii,1);
    jj=jj+1;
    end
end