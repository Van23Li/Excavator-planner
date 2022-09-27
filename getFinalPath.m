function nodes = getFinalPath(Close,cfg, Start,handles)
if nargin < 4
    handles = [];
    UI_flag = 0;
else
    UI_flag = 1;
end

%van
if UI_flag
    axes(handles.axes3); hold on;%xzk:创建坐标轴
else
    hold off
%     figure(33333)
end
for i = 1:length(Close)
    %             plot(Close(i).x,Close(i).y,'r*'); hold on
    quiver(Close(i).x,Close(i).y,-cos(Close(i).theta),-sin(Close(i).theta),'r');hold on
    axis equal
    axis([cfg.MINX cfg.MAXX cfg.MINY cfg.MAXY]);
end

wknode = Close(end); % RS曲线中最后一个元素是目标点

%van:
quiver(wknode.x,wknode.y,-cos(wknode.theta),-sin(wknode.theta),'b');hold on
axis equal
axis([cfg.MINX cfg.MAXX cfg.MINY cfg.MAXY]);

Close(end) = [];
nodes = [wknode];
% 找目标点wknode的parent,回溯，直到Close集合为空
%     while ~isempty(Close)
figure(222)
while wknode.x ~= Start(1) || wknode.y ~= Start(2) || wknode.theta ~= Start(3)
    n = length(Close);
    parent = wknode.parent;
    % 计算从目标返回到起始点的路径点序列，放入nodes中
    for i = n:-1:1
        flag = 0; % 只有赋值，没有使用
        tnode = Close(i);
        if tnode.xidx == parent(1)...
                && tnode.yidx == parent(2)...
                && tnode.yawidx == parent(3)
            
            %van:
            %                 plot(tnode.x,tnode.y,'bO'); hold on
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
    %         Close(i) = [];
end
end