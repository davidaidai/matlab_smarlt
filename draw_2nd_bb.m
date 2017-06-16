function handle=draw_2nd_bb(figure_handle,start_point,reference_point,theta,Rotation_matrix,length,draw_flag,delta,n,linehandle,color,r_backbone)
global r_2nd_bb;global number;
handle=zeros(1,number);
if strcmp(draw_flag,'compute')
    handle=linehandle;
else
if theta~=0
    %compute R_of_wire according to different values of n
    R_of_wire=length;
    switch n
       case 0,
        R_of_wire=R_of_wire-cos(n*2/3*pi-delta)*r_2nd_bb;
        case 1,
        R_of_wire=R_of_wire-cos(n*2/3*pi-delta)*r_2nd_bb;
        case 2,
        R_of_wire=R_of_wire-cos(n*2/3*pi-delta)*r_2nd_bb;
     end
% the coordinates in the actual body-attached frame
  start_point_a=Rotation_matrix'*(start_point-reference_point);
  circle_point_a=[start_point_a(1)+R_of_wire;start_point_a(2);start_point_a(3)];
  dtheta=theta/number;
  s_point=start_point;
   for i=1:number
   Angel=0+i*dtheta;
   temp_a=[circle_point_a(1)-R_of_wire*cos(Angel);0;circle_point_a(3)+R_of_wire*sin(Angel)];
   e_point=reference_point+Rotation_matrix*temp_a;
   x=[s_point(1);e_point(1)];
   y=[s_point(2);e_point(2)];
   z=[s_point(3);e_point(3)]; 
   s_point=e_point;
    if strcmp(draw_flag,'draw')
      linehandle(i)=line(x,y,z,'LineWidth',r_backbone);  
      set(linehandle(i),'color',color);
      handle(i)=linehandle(i);
    end
    if strcmp(draw_flag,'update')
      set(linehandle(i),'xdata',x);
      set(linehandle(i),'ydata',y);
      set(linehandle(i),'zdata',z);
      handle(i)=linehandle(i);
    end
   end
%     refreshdata(figure_handle);
else
    s_point=start_point;
    for i=1:number
       e_point=s_point+Rotation_matrix*[0;0;length/20]; 
       x=[s_point(1);e_point(1)];
       y=[s_point(2);e_point(2)];
       z=[s_point(3);e_point(3)];
       s_point=e_point;
       if strcmp(draw_flag,'draw')
          linehandle(i)=line(x,y,z,'LineWidth',r_backbone); 
          set(linehandle(i),'color',color);
          handle(i)=linehandle(i);
       end
       if strcmp(draw_flag,'update')
          set(linehandle(i),'xdata',x);
          set(linehandle(i),'ydata',y);
          set(linehandle(i),'zdata',z);
          handle(i)=linehandle(i);
       end
       
    end
%     refreshdata(figure_handle);
end
end
end