function [spacer_disk_points handle]=draw_1st_bb(figure_handle,start_point,reference_point,theta,Rotation_matrix,length,draw_flag,num_spacer_disk,linehandle,color,r_backbone)
%%% globale variables
% global r_backbone;
global number;
spacer_disk_points=zeros(3,num_spacer_disk);
j=0;
handle=zeros(1,number);
if strcmp(draw_flag,'compute')
    handle=linehandle;
else
if theta~=0
    R_of_wire=length;
   start_point_a=Rotation_matrix'*(start_point-reference_point);
   circle_point_a=[start_point_a(1)+R_of_wire;start_point_a(2);start_point_a(3)];
   dtheta=theta/number;
   s_point=start_point;
   for i=1:number
    Angel=0+i*dtheta;
   temp_a=[circle_point_a(1)-R_of_wire*cos(Angel);0;circle_point_a(3)+R_of_wire*sin(Angel)];
   e_point=reference_point+Rotation_matrix*temp_a;
     if mod(i,ceil(number/(num_spacer_disk+1)))==0
         j=j+1;
         spacer_disk_points(:,j)=e_point;
     end
   tempx=[s_point(1);e_point(1)];
   tempy=[s_point(2);e_point(2)];
   tempz=[s_point(3);e_point(3)];
   s_point=e_point;
   if strcmp(draw_flag,'draw')
   linehandle(i)=line(tempx,tempy,tempz,'LineWidth',r_backbone);
   set(linehandle(i),'color',color);
   handle(i)=linehandle(i);
   end
   if strcmp(draw_flag,'update')
    set(linehandle(i),'xdata',tempx);
    set(linehandle(i),'ydata',tempy);
    set(linehandle(i),'zdata',tempz);
    handle(i)=linehandle(i);
   end
   end
%     refreshdata(figure_handle);
else
    s_point=start_point;
    for i=1:number
        e_point=s_point+ Rotation_matrix*[0;0;length/number];
        if mod(i,ceil(number/(num_spacer_disk+1)))==0
         j=j+1;
         spacer_disk_points(:,j)=e_point;
        end
        tempx=[s_point(1);e_point(1)];
        tempy=[s_point(2);e_point(2)];
        tempz=[s_point(3);e_point(3)];
        s_point=e_point;
        if strcmp(draw_flag,'draw')
        linehandle(i)=line(tempx,tempy,tempz,'LineWidth',r_backbone); 
        set(linehandle(i),'color',color);
        handle(i)=linehandle(i);
        end
        if strcmp(draw_flag,'update')
        set(linehandle(i),'xdata',tempx);
        set(linehandle(i),'ydata',tempy);
        set(linehandle(i),'zdata',tempz);
        handle(i)=linehandle(i);
        end
    end
%     refreshdata(figure_handle);
end
end
   end