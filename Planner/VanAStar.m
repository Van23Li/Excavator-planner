function nodes = VanAStar(Start,End,Display,Configure,handles)
if nargin < 5
    handles = [];
    UI_flag = 0;
else
    UI_flag = 1;
end

cfg = Configure;
mres = cfg.MOTION_RESOLUTION; % [m] path interporate resolution

% translate (x,y,theta) to the index of grid
[isok,xidx,yidx,thidx] = CalcIdx(cfg,Start(1),Start(2),Start(3));
if isok
    tnode = Node(xidx,yidx,thidx,mres,0,Start(1),Start(2),Start(3),[xidx,yidx,thidx],0);
end

Open = [tnode];
Close = [];
if UI_flag
    axes(handles.axes2);
end

if Display
    figure(1);
    rectangle('Position',[cfg.ObstPoint(1,1),cfg.ObstPoint(2,2),cfg.ObstPoint(2,1)-cfg.ObstPoint(1,1),cfg.ObstPoint(1,2)-cfg.ObstPoint(2,2)],'EdgeColor','r');
    
    axis equal
    xlim([cfg.MINX,cfg.MAXX]);
    ylim([cfg.MINY,cfg.MAXY]);
end
while ~isempty(Open)
    if Display
        hold on
        for i = 1:length(Open)
            figure(1)
            quiver(Open(i).x,Open(i).y,-cos(Open(i).theta),-sin(Open(i).theta),'g');hold on
            drawnow
            if UI_flag
                pause(0.001);
            end
        end
    end
    [isopenFlag,Id] = isopen(End,Open,cfg);
    if isopenFlag
        if UI_flag
            set(handles.textedit,'String','Find Goal!!');
        else
            disp('Find Goal!!');
        end
        Close(end+1) = Open(Id);
        nodes = getFinalPath(Close,cfg, Start);
        break;
    end
    
    % pop the least cost node from open to close
    [wknode,Open] = PopNode(cfg, Open);
    [isok,idx] = inNodes(wknode,Close);
    if isok
        Close(idx) = wknode;
    else
        Close = [Close, wknode];
    end
    
    [Open,Close] = Update(wknode,Open,Close,cfg);
    if Display
        figure(1)
        quiver(wknode.x,wknode.y,-cos(wknode.theta),-sin(wknode.theta),'k');hold on
        drawnow
    end
end
end
%%
function [wknode,nodes] = PopNode(cfg, nodes)
% pop the least cost node from open to close
mincost = inf;
minidx = 1;
for idx = 1:length(nodes)
    tnode = nodes(idx);
    tcost = TotalCost(tnode,cfg);
    if tcost < mincost
        mincost = tcost;
        minidx = idx;
    end
end
wknode = nodes(minidx);
nodes(minidx) = [];
end
%%
function cost = TotalCost(wknode,cfg)
gres = cfg.XY_GRID_RESOLUTION;
costmap = cfg.ObstMap;

cost = cfg.AStar_COST * costmap(wknode.yidx,wknode.xidx);

xshift = wknode.x - (gres*(wknode.xidx-0.5)+cfg.MINX);
yshift = wknode.y - (gres*(wknode.yidx-0.5)+cfg.MINY);
cost = cost + cfg.Node_COST * norm([xshift,yshift]);

cost = cost + cfg.ANGLE_COST * min(abs(wknode.theta - cfg.End(3)), abs(cfg.End(3)-wknode.theta));

cost = wknode.cost + cfg.H_COST * cost;
end
%%
function [isok,idx] = inNodes(node,nodes)
for i = 1:length(nodes)
    tnode = nodes(i);
    if node.xidx == tnode.xidx...
            && node.yidx == tnode.yidx...
            && node.yawidx == tnode.yawidx
        idx = i;
        isok = true;
        return
    end
end
idx = 1;
isok = false;
end
%%
function [Open,Close] = Update(wknode,Open,Close,cfg)
sres = pi/2/cfg.N_STEER;
for D = [-1,1]
    for delta = [-pi/2:sres:-sres,0,sres:sres:pi/2]
        [results, flag, ~] = ik(cfg.dhparams, wknode.x, wknode.y, wknode.theta+pi+cfg.params.zeta,cfg.params);
        if flag == 1
            [flag, flag_p, flag_n] = exam_reach(results, cfg.params_c);
            if flag == 1
                lambda_p_1 = 0; lambda_p_2 = 0; lambda_p_3 = 0;
                lambda_n_1 = 0; lambda_n_2 = 0; lambda_n_3 = 0;
                if flag_p == 1
                    [lambda_p_1, lambda_p_2, lambda_p_3] = work2drive(results(1,1), results(1,2), results(1,3), cfg.params_c);
                end
                if flag_n == 1
                    [lambda_n_1, lambda_n_2, lambda_n_3] = work2drive(results(2,1), results(2,2), results(2,3), cfg.params_c);
                end
                [isok,tnode] = CalcNextNode(wknode,D,delta,cfg,lambda_p_1, lambda_p_2, lambda_p_3,lambda_n_1, lambda_n_2, lambda_n_3);
            else
                isok = false;
            end
        end
        if isok == false
            continue
        end
        
        for i = 1:length(tnode)
            [isok,~] = inNodes(tnode(i),Close);
            if isok
                continue
            end
            
            [isok,idx] = inNodes(tnode(i),Open);
            if isok
                tcost = TotalCost(tnode(i),cfg);
                ttnode = Open(idx);
                ttcost = TotalCost(ttnode,cfg);
                if tcost < ttcost
                    Open(idx) = tnode(i);
                end
            else
                Open(end+1) = tnode(i);
            end
        end
    end
end
end
%%
function [isok,tnode] = CalcNextNode(wknode,D,delta,cfg,lambda1_p_1, lambda1_p_2, lambda1_p_3,lambda1_n_1, lambda1_n_2, lambda1_n_3)
px = wknode.x;
py = wknode.y;
theta = wknode.theta;
% yawidx = wknode.yawidx;
gres = cfg.XY_GRID_RESOLUTION;
% obstline = cfg.ObstLine;
length = gres*sqrt(2);
obstpoint = cfg.ObstPoint;

tnode(1,:) = wknode;
[px,py,pth,isok] = MapDynamic(px,py,theta,D,delta,length,cfg);
j = 0;
if isok
    for i = 1:size(pth,2)
        tvec = [px,py,pth(i)];
        isCollision = CollisionCheck(tvec,obstpoint,cfg);
        
        if isCollision
            isok = false;
            return
        else
            j=j+1;
            [isok,xidx,yidx,thidx] = CalcIdx(cfg,px,py,pth(i));
            if isok == false
                return
            else
                cost = wknode.cost;
                if D > 0
                    cost = cost + cfg.FORW_COST*gres*sqrt(2);
                else
                    cost = cost + cfg.BACK_COST*gres*sqrt(2);
                end
                if D ~= wknode.D
                    cost = cost + cfg.SB_COST;
                end
                cost = cost + cfg.STEER_COST*abs(delta);
                cost = cost + cfg.STEER_CHANGE_COST * abs(delta-wknode.delta);
                
                [results2, flag2, ~] = ik(cfg.dhparams, px, py, pth(i)+pi+cfg.params.zeta,cfg.params);
                
                lambda2_p_1 = 0; lambda2_p_2 = 0; lambda2_p_3 = 0;
                lambda2_n_1 = 0; lambda2_n_2 = 0; lambda2_n_3 = 0;
                if flag2 == 1
                    [flag2, flag2_p, flag2_n] = exam_reach(results2, cfg.params_c);
                    if flag2 == 1
                        if flag2_p == 1
                            [lambda2_p_1, lambda2_p_2, lambda2_p_3] = work2drive(results2(1,1), results2(1,2), results2(1,3), cfg.params_c);
                        end
                        if flag2_n == 1
                            [lambda2_n_1, lambda2_n_2, lambda2_n_3] = work2drive(results2(2,1), results2(2,2), results2(2,3), cfg.params_c);
                        end
                    end
                end
                lambda1 = (abs(lambda1_p_1 - lambda2_p_1) + abs(lambda1_n_1 - lambda2_n_1))/2;
                lambda2 = (abs(lambda1_p_2 - lambda2_p_2) + abs(lambda1_n_2 - lambda2_n_2))/2;
                lambda3 = (abs(lambda1_p_3 - lambda2_p_3) + abs(lambda1_n_3 - lambda2_n_3))/2;
                
                cost = cost + cfg.Drive_COST*(lambda1+lambda2+lambda3)/1000;
                tnode(j,:) = Node(xidx,yidx,thidx,D,delta,px,py,pth(i),...
                    [wknode.xidx,wknode.yidx,wknode.yawidx],cost);
            end
        end
    end
end
end
%%
function [x,y,theta,flag] = MapDynamic(x,y,theta,D,delta,L,cfg)
x = x+D*L*cos(theta+delta);
y = y+D*L*sin(theta+delta);
[~,xidx,yidx,~] = CalcIdx(cfg,x,y);
[theta,flag] = MapTheta(xidx, yidx, cfg, theta);
if flag
    theta = wrapToPi(theta);
end
end
%%
function [Theta,flag]= MapTheta(xidx, yidx, cfg, theta)
MAP = cfg.Map;
flag = 0;
Theta = [];
for i = 1:size(MAP,3)
    if MAP(xidx,yidx,i)==1
        if abs(-pi + wrapToPi(cfg.YAW_GRID_RESOLUTION_SEARCH*(i-1)) + pi - theta)<= cfg.MAXTHETA
            Theta = [Theta, -pi + wrapToPi(cfg.YAW_GRID_RESOLUTION_SEARCH*(i-1)) + pi];
            flag = 1;
        end
        
    end
end
end
%%
function [isok,xidx,yidx,thidx] = CalcIdx(cfg,x,y,theta)
if nargin < 4
    thidx = Inf;
else
    yawres = cfg.YAW_GRID_RESOLUTION;
    theta = wrapToPi(theta);
    thidx = ceil((theta-cfg.MINYAW)/yawres);
end

gres = cfg.XY_GRID_RESOLUTION;
xidx = ceil((x-cfg.MINX)/gres);
yidx = ceil((y-cfg.MINY)/gres);
isok = true;
if xidx <=0 || xidx > ceil((cfg.MAXX-cfg.MINX)/gres)
    isok = false;
    return
elseif yidx <=0 || yidx > ceil((cfg.MAXY-cfg.MINY)/gres)
    isok = false;
    return
end
costmap = cfg.ObstMap;
if costmap(yidx,xidx) == inf
    isok = false;
end
end
%%
function [isopenFlag,Id] = isopen(node,Open,cfg)
isopenFlag = 0;
Id = 0;

if  isempty(Open)
    isopenFlag = 0;
else
    for i = 1:length(Open)
        if abs((node(1) - Open(i).x)) <= cfg.TolLen...
                && abs((node(2) - Open(i).y)) <= cfg.TolLen...
                && abs((node(3) - Open(i).theta )) <= cfg.TolAng
            isopenFlag = 1;
            Id = i;
            return;
        end
    end
end

end
%%