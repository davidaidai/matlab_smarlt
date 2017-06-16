function handle=draw_gripper(figure_handle,center_1,axis_x,center_2,color,transprancy,draw_flag,gripper_handle)

handle=zeros(1,3);
figure(figure_handle);
[tmpm tmpn]=size(color);
if tmpm<tmpn
    color=color';
end
tmpX=axis_x-center_1;radius=norm(tmpX);tmpX=tmpX/radius;
tmpZ=center_2-center_1;height=norm(tmpZ);tmpZ=tmpZ/height;
tmpY=cross(tmpZ,tmpX);

vertex=[center_1+0.2*tmpX*radius center_1+tmpX*radius center_1+tmpX*radius+.45*tmpZ*height center_1+0.2*tmpX*radius+tmpZ*height];
vertex=[vertex center_1-0.2*tmpX*radius center_1-tmpX*radius center_1-tmpX*radius+.45*tmpZ*height center_1-0.2*tmpX*radius+tmpZ*height];
vertex=[vertex center_1 center_1+0.5*tmpZ*height center_1+0.5*tmpY*radius];

if strcmp(draw_flag,'draw')
    handle(1)=patch(vertex(1,1:4)',vertex(2,1:4)',vertex(3,1:4)',32);
    set(handle(1),'facecolor',color,'FaceAlpha',transprancy);
    handle(2)=patch(vertex(1,5:8)',vertex(2,5:8)',vertex(3,5:8)',32);
    set(handle(2),'facecolor',color,'FaceAlpha',transprancy);
    handle(3)=patch(vertex(1,9:11)',vertex(2,9:11)',vertex(3,9:11)',32);
    set(handle(3),'facecolor',color,'FaceAlpha',transprancy); 
end

if strcmp(draw_flag,'update')    
    
    set(gripper_handle(1),'XData',vertex(1,1:4)');
    set(gripper_handle(1),'YData',vertex(2,1:4)');
    set(gripper_handle(1),'ZData',vertex(3,1:4)');
    
    set(gripper_handle(2),'XData',vertex(1,5:8)');
    set(gripper_handle(2),'YData',vertex(2,5:8)');
    set(gripper_handle(2),'ZData',vertex(3,5:8)');
    
    set(gripper_handle(3),'XData',vertex(1,9:11)');
    set(gripper_handle(3),'YData',vertex(2,9:11)');
    set(gripper_handle(3),'ZData',vertex(3,9:11)');
    
    handle=gripper_handle;
%     refreshdata(figure_handle);
end




