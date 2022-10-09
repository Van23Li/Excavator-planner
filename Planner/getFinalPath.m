function nodes = getFinalPath(Close,cfg, Start,handles)
if nargin < 4
    handles = [];
    UI_flag = 0;
else
    UI_flag = 1;
end

%van
if UI_flag
    axes(handles.axes3); hold on;
end

wknode = Close(end);

Close(end) = [];
nodes = [wknode];

figure(2)
rectangle('Position',[cfg.ObstPoint(1,1),cfg.ObstPoint(2,2),cfg.ObstPoint(2,1)-cfg.ObstPoint(1,1),cfg.ObstPoint(1,2)-cfg.ObstPoint(2,2)],'EdgeColor','r');
while wknode.x ~= Start(1) || wknode.y ~= Start(2) || wknode.theta ~= Start(3)
    n = length(Close);
    parent = wknode.parent;
    for i = n:-1:1
        flag = 0;
        tnode = Close(i);
        if tnode.xidx == parent(1)...
                && tnode.yidx == parent(2)...
                && tnode.yawidx == parent(3)
            
            quiver(tnode.x,tnode.y,-cos(tnode.theta),-sin(tnode.theta),'b');hold on
            axis equal
            axis([cfg.MINX cfg.MAXX cfg.MINY cfg.MAXY]);
            drawnow
            if UI_flag
                pause(0.5);
            end
            nodes(end+1) = tnode;
            wknode = tnode;
            flag = 1;
            break
        end
    end
end
end