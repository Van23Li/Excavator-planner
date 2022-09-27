function sector_len(x0,y0,angle1,angle2, r,step_phi)
%基于各角度的权重，插值产生不同角度方向上的半径
%%
% %(x0,y0)为扇形原点，(angle1,angle2)为扇形张角, r是半径
% t=linspace(angle1,angle2,100);
% x=r*cos(t);y=r*sin(t);
% fill([x0 x],[y0 y],'r')
% axis([x0-1 x0+1 y0-1 y0+1])

r0 = [x0,y0];%圆心
% radius = r;
if sum(r~=0)==1
    t=linspace(angle1,angle2,100);
    x0 = r0(1)+r(1)*cos(t);
    y0 = r0(2)+r(1)*sin(t);
else
    r_x = 1:size(r,2);
    radius = spline(r_x,[r],linspace(1,size(r,2),100));
    
    t=linspace(angle1,angle2,100);
    x0 = r0(1)+radius.*cos(t);
    y0 = r0(2)+radius.*sin(t);
end
set(patch,'faceColor','c','xdata',[r0(1),x0],'ydata',[r0(2),y0]);
end