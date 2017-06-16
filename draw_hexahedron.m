function handle=draw_hexahedron(figure_handle,face_vertex,color,transprancy,draw_flag,box_handle)
% face_vertex is 3 by 8, first 4 columns for bottom and the rest for top
%  4  ========== 3    8 ========== 7
%     |        |        |        |
%     |        |        |        |
%     |        |        |        |
%  1  ========== 2    5 ========== 6
handle=zeros(1,6);
figure(figure_handle);
[tmpm tmpn]=size(color);
if tmpm<tmpn
    color=color';
end
if strcmp(draw_flag,'draw')
    handle(1)=patch(face_vertex(1,1:4)',face_vertex(2,1:4)',face_vertex(3,1:4)',32);
    set(handle(1),'facecolor',color,'FaceAlpha',transprancy);
    handle(2)=patch(face_vertex(1,5:8)',face_vertex(2,5:8)',face_vertex(3,5:8)',32);
    set(handle(2),'facecolor',color,'FaceAlpha',transprancy);
    vertex=[face_vertex(:,1) face_vertex(:,2) face_vertex(:,6) face_vertex(:,5)];
    handle(3)=patch(vertex(1,:)',vertex(2,:)',vertex(3,:)',32);
    set(handle(3),'facecolor',color,'FaceAlpha',transprancy);
    vertex=[face_vertex(:,2) face_vertex(:,3) face_vertex(:,7) face_vertex(:,6)];
    handle(4)=patch(vertex(1,:)',vertex(2,:)',vertex(3,:)',32);
    set(handle(4),'facecolor',color,'FaceAlpha',transprancy);
    vertex=[face_vertex(:,3) face_vertex(:,4) face_vertex(:,8) face_vertex(:,7)];
    handle(5)=patch(vertex(1,:)',vertex(2,:)',vertex(3,:)',32);
    set(handle(5),'facecolor',color,'FaceAlpha',transprancy);
    vertex=[face_vertex(:,1) face_vertex(:,4) face_vertex(:,8) face_vertex(:,5)];
    handle(6)=patch(vertex(1,:)',vertex(2,:)',vertex(3,:)',32);
    set(handle(6),'facecolor',color,'FaceAlpha',transprancy);
end

if strcmp(draw_flag,'update')    
    set(box_handle(1),'XData',face_vertex(1,1:4)');
    set(box_handle(1),'YData',face_vertex(2,1:4)');
    set(box_handle(1),'ZData',face_vertex(3,1:4)');
    
    set(box_handle(2),'XData',face_vertex(1,5:8)');
    set(box_handle(2),'YData',face_vertex(2,5:8)');
    set(box_handle(2),'ZData',face_vertex(3,5:8)');
    
    vertex=[face_vertex(:,1) face_vertex(:,2) face_vertex(:,6) face_vertex(:,5)];
    set(box_handle(3),'XData',vertex(1,1:4)');
    set(box_handle(3),'YData',vertex(2,1:4)');
    set(box_handle(3),'ZData',vertex(3,1:4)');
    
    vertex=[face_vertex(:,2) face_vertex(:,3) face_vertex(:,7) face_vertex(:,6)];
    set(box_handle(4),'XData',vertex(1,1:4)');
    set(box_handle(4),'YData',vertex(2,1:4)');
    set(box_handle(4),'ZData',vertex(3,1:4)');
    
    vertex=[face_vertex(:,3) face_vertex(:,4) face_vertex(:,8) face_vertex(:,7)];
    set(box_handle(5),'XData',vertex(1,1:4)');
    set(box_handle(5),'YData',vertex(2,1:4)');
    set(box_handle(5),'ZData',vertex(3,1:4)');
    
    vertex=[face_vertex(:,1) face_vertex(:,4) face_vertex(:,8) face_vertex(:,5)];
    set(box_handle(6),'XData',vertex(1,1:4)');
    set(box_handle(6),'YData',vertex(2,1:4)');
    set(box_handle(6),'ZData',vertex(3,1:4)');
    
    handle=box_handle;
%     refreshdata(figure_handle);
end




