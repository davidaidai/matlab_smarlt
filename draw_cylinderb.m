function handle=draw_cylinderb(figure_handle,center_1,axis_x,center_2,color,transprancy,draw_flag,cylinder_handle)
number=16;
handle=zeros(1,number+2);
figure(figure_handle);
%%%the 3rd condition, only for compution
if strcmp(draw_flag,'compute') 
  handle=cylinder_handle;
end
[tmpm tmpn]=size(color);
if tmpm<tmpn
    color=color';
end
tmpX=axis_x-center_1;radius=norm(tmpX);tmpX=tmpX/radius;
tmpZ=center_2-center_1;height=norm(tmpZ);tmpZ=tmpZ/height;
tmpY=cross(tmpZ,tmpX);
theta=0:2*pi/number:2*pi;
vertex=zeros(3,number*2+2);    
for index=1:number+1
    vertex(:,index)=radius*cos(theta(index))*tmpX+radius*sin(theta(index))*tmpY+center_1;
    vertex(:,index+number+1)=vertex(:,index)+height*tmpZ;
end
if strcmp(draw_flag,'draw')
    handle(1)=patch(vertex(1,1:number+1)',vertex(2,1:number+1)',vertex(3,1:number+1)',32);
    set(handle(1),'facecolor',color,'FaceAlpha',transprancy);
    handle(2)=patch(vertex(1,number+2:2*number+2)',vertex(2,number+2:2*number+2)',vertex(3,number+2:2*number+2)',32);
    set(handle(2),'facecolor',color,'FaceAlpha',transprancy);
    
    for index=1:1:number
        tmp_vertex=[vertex(:,index) vertex(:,index+1) vertex(:,number+index+2) vertex(:,number+index+1)];
        handle(index+2)=patch(tmp_vertex(1,:)',tmp_vertex(2,:)',tmp_vertex(3,:)',32);
        set(handle(index+2),'facecolor',color,'FaceAlpha',transprancy,'EdgeAlpha',0);
    end
    
end

if strcmp(draw_flag,'update')    
    
    set(cylinder_handle(1),'XData',vertex(1,1:number+1)');
    set(cylinder_handle(1),'YData',vertex(2,1:number+1)');
    set(cylinder_handle(1),'ZData',vertex(3,1:number+1)');
    
    set(cylinder_handle(2),'XData',vertex(1,number+2:2*number+2)');
    set(cylinder_handle(2),'YData',vertex(2,number+2:2*number+2)');
    set(cylinder_handle(2),'ZData',vertex(3,number+2:2*number+2)');
    
    for index=1:1:number
        tmp_vertex=[vertex(:,index) vertex(:,index+1) vertex(:,number+index+2) vertex(:,number+index+1)];
        set(cylinder_handle(index+2),'XData',tmp_vertex(1,1:4)');
        set(cylinder_handle(index+2),'YData',tmp_vertex(2,1:4)');
        set(cylinder_handle(index+2),'ZData',tmp_vertex(3,1:4)');
    end
    
    handle=cylinder_handle;
%     refreshdata(figure_handle);
    
end




