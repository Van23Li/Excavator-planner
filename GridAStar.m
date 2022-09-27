function costmap = GridAStar(cfg,goal)
%计算cost,障碍物处为inf
obstlist = cfg.ObstList;
gres = cfg.XY_GRID_RESOLUTION;

[minx,miny,obmap] = CalcObstMap(obstlist,gres,cfg);%注意，obmap上下颠倒，左右不变
col = goal(1);
row = goal(2);
angle = goal(3);
col = ceil((col-minx)/gres);
row = ceil((row-miny)/gres);
angle = ceil((angle-cfg.MINYAW)/cfg.YAW_GRID_RESOLUTION);
costmap = 0*obmap;
if cfg.AStarStyle == 2
    goal = [row,col];
    dim = size(obmap);
    for i = 1:dim(1)
        for j = 1:dim(2)
            fprintf(['i=', num2str(i), ' in total ', num2str(dim(1)),'; j=', num2str(j), ' in total ', num2str(dim(2)),'\n' ])
            if obmap(i,j) == 1
                costmap(i,j) = inf;
                continue
            elseif i == col && j == row
                continue
            end
            start = [i,j];
            cost = AStarSearch(start,goal,obmap);
            costmap(i,j) = cost;
        end
    end
elseif cfg.AStarStyle == 3
    goal = [row,col,angle];
    dim = size(obmap);
    for i = 1:dim(1)
        for j = 1:dim(2)
            fprintf(['i=', num2str(i), ' in total ', num2str(dim(1)),'; j=', num2str(j), ' in total ', num2str(dim(2)),'\n' ])
            for k = 1:dim(3)
                if obmap(i,j,k) == 1
                    costmap(i,j,k) = inf;
                    continue
                elseif i == col && j == row && k == angle
                    continue
                end
                start = [i,j,k];
                cost = AStarSearch_3d(start,goal,obmap);
                costmap(i,j,k) = cost;
            end
        end
    end
    
end
end
%%
function [minx,miny,obmap] = CalcObstMap(obstlist,gres,cfg)
%按照分辨率构建地图，距离障碍物距离小于D（0.5）的置1，不可行区域置1，否则置0
% minx = min(obstlist(:,1));
% maxx = max(obstlist(:,1));
% miny = min(obstlist(:,2));
% maxy = max(obstlist(:,2));
minx = cfg.MINX;
maxx = cfg.MAXX;
miny = cfg.MINY;
maxy = cfg.MAXY;
xwidth = maxx - minx;
xwidth = ceil(xwidth/gres);
ywidth = maxy - miny;
ywidth = ceil(ywidth/gres);
thetawidth = 20;
if cfg.AStarStyle == 2
    obmap = zeros(ywidth,xwidth);
    for i = 1:ywidth
        for j = 1:xwidth
            if sum(cfg.Map(j,i,:))==0
                obmap(i,j) = 1;
                continue
            else
                ix = minx+(j-1/2)*gres;
                iy = miny+(i-1/2)*gres; %grid的中点
                [~,D] = knnsearch(obstlist,[ix,iy]);    %寻找离起点[ix,iy]最近的障碍物表面
                
                if D < 0.3
                    obmap(i,j) = 1;
                end
            end
        end
    end
elseif cfg.AStarStyle == 3
    obmap = zeros(ywidth,xwidth,thetawidth);
    for i = 1:ywidth
        for j = 1:xwidth
            for k = 1:thetawidth
                if cfg.Map(j,i,k)==0
                    obmap(i,j,k) = 1;
                    continue
                else
                    ix = minx+(j-1/2)*gres;
                    iy = miny+(i-1/2)*gres; %grid的中点
                    [~,D] = knnsearch(obstlist,[ix,iy]);    %寻找离起点[ix,iy]最近的障碍物表面
                    
                    if D < 0.3
                        obmap(i,j,k) = 1;
                    end
                end
            end
        end
    end
end
end
%%
function cost = AStarSearch(start,goal,obmap)
dim = size(obmap);
Grids = zeros(dim(1),dim(2),4);
for i = 1:dim(1)
    for j = 1:dim(2)
        Grids(i,j,1) = i; % 父节点的所在行
        Grids(i,j,2) = j; % 父节点的所在列
        Grids(i,j,3) = norm(([i,j]-goal)); % 启发值h
        Grids(i,j,4) = inf; % g值
    end
end
Open = [start];
Grids(start(1),start(2),4) = 0;
Close = [];
while ~isempty(Open)
    [wknode,Open] = PopOpen(Open,Grids);
    [Grids,Open,Close] = Update(wknode,goal,obmap,Grids,Open,Close);
    Close(end+1,:) = wknode;
end
cost = Grids(goal(1),goal(2),3)+Grids(goal(1),goal(2),4);
end
%%
function [wknode,Open] = PopOpen(Open,Grids)
mincost = inf;
minidx = 1;
for i = 1:size(Open,1)
    node = Open(i,:);
    tcost = Grids(node(1),node(2),3)+Grids(node(1),node(2),4);
    if tcost < mincost
        minidx = i;
        mincost = tcost;
    end
end
wknode = Open(minidx,:);
Open(minidx,:) = [];
end
%%
function [Grids,Open,Close] = Update(wknode,goal,obmap,Grids,Open,Close)
dim = size(obmap);
for i = -1:1
    for j = -1:1
        adjnode = wknode+[i,j];
        row = adjnode(1);
        col = adjnode(2);
        if i == 0 && j == 0
            continue
        elseif row < 1 || row > dim(1)
            continue
        elseif col < 1 || col > dim(2)
            continue
        elseif obmap(row,col) == 1
            continue
        end
        tcost = Grids(wknode(1),wknode(2),4)+norm([i,j]);
        if Grids(row,col,4) > tcost
            Grids(row,col,1) = wknode(1);
            Grids(row,col,2) = wknode(2);
            Grids(row,col,4) = tcost;
            % add adjnode to Open except wknode is goal
            if ~ismember(adjnode,Open,'rows') && ~isequal(adjnode,goal)
                Open(end+1,:) = adjnode;
            end
            % if adjnode is in Close remove it
            if isempty(Close)
                % do nothing
            elseif ismember(adjnode,Close,'rows')
                [~,rIdx] = ismember(adjnode,Close,'rows');
                Close(rIdx,:) = [];
            end
        end
    end
end
end
%%
function cost = AStarSearch_3d(start,goal,obmap)
dim = size(obmap);
Grids = zeros(dim(1),dim(2),4);
for i = 1:dim(1)
    for j = 1:dim(2)
        for k = 1:dim(3)
            Grids(i,j,k,1) = i; % 父节点的所在行
            Grids(i,j,k,2) = j; % 父节点的所在列
            Grids(i,j,k,3) = norm(([i,j,k]-goal)); % 启发值h
            Grids(i,j,k,4) = inf; % g值
            Grids(i,j,k,5) = j; % 父节点的所在角度
        end
    end
end
Open = [start];
Grids(start(1),start(2),start(3),4) = 0;
Close = [];
while ~isempty(Open)
    [wknode,Open] = PopOpen_3d(Open,Grids);
    [Grids,Open,Close] = Update_3d(wknode,goal,obmap,Grids,Open,Close);
    Close(end+1,:) = wknode;
end
cost = Grids(goal(1),goal(2),goal(3),3)+Grids(goal(1),goal(2),goal(3),4);
end
%%
function [wknode,Open] = PopOpen_3d(Open,Grids)
mincost = inf;
minidx = 1;
for i = 1:size(Open,1)
    node = Open(i,:);
    tcost = Grids(node(1),node(2),node(3),3)+Grids(node(1),node(2),node(3),4);
    if tcost < mincost
        minidx = i;
        mincost = tcost;
    end
end
wknode = Open(minidx,:);
Open(minidx,:) = [];
end
%%
function [Grids,Open,Close] = Update_3d(wknode,goal,obmap,Grids,Open,Close)
dim = size(obmap);
for i = -1:1
    for j = -1:1
        for k = -1:1
            adjnode = wknode+[i,j,k];
            row = adjnode(1);
            col = adjnode(2);
            angle = adjnode(3);
            if i == 0 && j == 0 && k == 0
                continue
            elseif row < 1 || row > dim(1)
                continue
            elseif col < 1 || col > dim(2)
                continue
            elseif angle < 1 || angle > dim(3)
                continue
            elseif obmap(row,col,angle) == 1
                continue
            end
            tcost = Grids(wknode(1),wknode(2),wknode(3),4)+norm([i,j,k]);
            if Grids(row,col,angle,4) > tcost
                Grids(row,col,angle,1) = wknode(1);
                Grids(row,col,angle,2) = wknode(2);
                Grids(row,col,angle,5) = wknode(3);
                Grids(row,col,angle,4) = tcost;
                % add adjnode to Open except wknode is goal
                if ~ismember(adjnode,Open,'rows') && ~isequal(adjnode,goal)
                    Open(end+1,:) = adjnode;
                end
                % if adjnode is in Close remove it
                if isempty(Close)
                    % do nothing
                elseif ismember(adjnode,Close,'rows')
                    [~,rIdx] = ismember(adjnode,Close,'rows');
                    Close(rIdx,:) = [];
                end
            end
        end
    end
end
end