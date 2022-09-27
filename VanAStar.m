function nodes = VanAStar(Start,End,Configure,handles)
if nargin < 4
    handles = [];
    UI_flag = 0;
else
    UI_flag = 1;
end

cfg = Configure;

mres = cfg.MOTION_RESOLUTION; % 0.1; [m] path interporate resolution

% 把起始的位姿(x,y,theta)转换为grid上的栅格索引
[isok,xidx,yidx,thidx] = CalcIdx(cfg,Start(1),Start(2),Start(3));

if isok % 把位姿栅格定义为一个结点，形成链表结构
    tnode = Node(xidx,yidx,thidx,mres,0,Start(1),Start(2),Start(3),[xidx,yidx,thidx],0);
    % [xidx,yidx,thidx]是父节点
end

Open = [tnode]; % hybrid A*的Open集合
Close = [];
x = [];
y = [];
th = [];
D = [];
delta = [];
if UI_flag
    axes(handles.axes2);%xzk:创建坐标轴
end

while ~isempty(Open)
    %Van: 可视化Open
    hold on
    for i = 1:length(Open)
        %         plot(Open(i).x,Open(i).y,'g*'); hold on
        quiver(Open(i).x,Open(i).y,-cos(Open(i).theta),-sin(Open(i).theta),'g');hold on
        xlim([cfg.MINX,cfg.MAXX]);
        ylim([cfg.MINY,cfg.MAXY]);
        if UI_flag
            pause(0.001);
        end
    end
    %判断目标点是否出现在open列表中
    [isopenFlag,Id]=isopen(End,Open,cfg);
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
    
    %     drawnow
    
    % pop the least cost node from open to close
    [wknode,Open] = PopNode(cfg, Open);
    [isok,idx] = inNodes(wknode,Close);
    
    %Van: 可视化剔除的node
    %     if length(Open) == 1
    %     plot(wknode.x,wknode.y,'k*'); hold on
    quiver(wknode.x,wknode.y,-cos(wknode.theta),-sin(wknode.theta),'k');hold on
    axis equal
    axis([cfg.MINX cfg.MAXX cfg.MINY cfg.MAXY]);
    drawnow
    
    % 判断是否在Close集合内
    if isok
        Close(idx) = wknode;
    else
        Close = [Close, wknode];
    end
    
    [Open,Close] = Update(wknode,Open,Close,cfg); % 使用
    
end
end
%%
function [wknode,nodes] = PopNode(cfg, nodes)
% pop the least cost node from open to close
mincost = inf;
minidx = 1;
% gres = cfg.XY_GRID_RESOLUTION;
for idx = 1:length(nodes)
    tnode = nodes(idx);
    % x in the col y in row
    % tcost为A*计算出的预期花费加上起点到Grid中心的距离
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
% 计算总代价，f = g + h
% h = A*算法计算出的代价 + 从当前结点到栅格中心的代价 + 当前角度到目标角度的代价
% g = 已走过的代价
gres = cfg.XY_GRID_RESOLUTION;
costmap = cfg.ObstMap;

% A*算法计算出的代价
% aaa = cfg.AStar_COST * costmap(wknode.yidx,wknode.xidx)
cost = cfg.AStar_COST * costmap(wknode.yidx,wknode.xidx); % 无障碍碰撞地图上的成本，用A*搜出来的，二维地图，没用航向

% 从当前结点到栅格中心的代价
xshift = wknode.x - (gres*(wknode.xidx-0.5)+cfg.MINX); % 栅格的index是线的交点，而不是栅格的中心,在求坐标时所以会有减0.5
yshift = wknode.y - (gres*(wknode.yidx-0.5)+cfg.MINY);
%     cost = cost+cfg.H_COST*norm([xshift,yshift]);
% bbb=cfg.Node_COST * norm([xshift,yshift])
cost = cost + cfg.Node_COST * norm([xshift,yshift]);

% 当前角度到目标角度的代价
% ccc=cfg.ANGLE_COST * min(abs(wknode.theta - cfg.End(3)), abs(cfg.End(3)-wknode.theta))
cost = cost + cfg.ANGLE_COST * min(abs(wknode.theta - cfg.End(3)), abs(cfg.End(3)-wknode.theta));

% f = g + h
cost = wknode.cost + cfg.H_COST * cost;
end
%%
function [isok,idx] = inNodes(node,nodes)
% 查看node十是否在nodes内
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
%     mres = cfg.MOTION_RESOLUTION; % motino resolution
%     smax = cfg.MAX_STEER; % 0.6[rad],maximum steering angle
sres = pi/2/cfg.N_STEER; % 20,steering resolution
% sres = cfg.YAW_GRID_RESOLUTION_SEARCH;
% all possible control input，
for D = [-1,1] % D的正负代表前进或后退
    for delta = [-pi/2:sres:-sres,0,sres:sres:pi/2] % delta是转向角，分辨率是0.05[rad],有21个子结点(包含0[rads])
        [results, flag, result_num] = ik(cfg.dhparams, wknode.x, wknode.y, wknode.theta+pi+cfg.params.zeta,cfg.params);
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
                [isok,tnode] = CalcNextNode(wknode,D,delta,cfg,lambda_p_1, lambda_p_2, lambda_p_3,lambda_n_1, lambda_n_2, lambda_n_3); % 计算wknode的所有子结点，一共2*21=42个，此函数是根据固定的D和delta计算wknode沿着一条路径的所有子结点，tnode是此条路径的末端点
            else
                isok = false
            end
        end
        if isok == false % 子结点不可行
            continue
        end
        
        for i = 1:length(tnode)
            [isok,~] = inNodes(tnode(i),Close);% 在Close集合中
            if isok
                continue
            end
            % 拓展的节点如果在Open中比较f值;若不在则添加到Open中
            [isok,idx] = inNodes(tnode(i),Open);
            if isok
                % 与之前的cost比较，进行更新
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
% 计算wknode的所有子结点，一共2*21=42个
% 此函数是根据固定的D和delta计算wknode沿着一条路径的所有子结点，tnode是此条路径的末端点
px = wknode.x;
py = wknode.y;
theta = wknode.theta;
yawidx = wknode.yawidx;
gres = cfg.XY_GRID_RESOLUTION;
obstline = cfg.ObstLine;
length = gres*sqrt(2);
obstpoint = cfg.ObstPoint;

tnode(1,:) = wknode;
% if D >0
%     a=2
% end
[px,py,pth,isok] = MapDynamic(px,py,theta,D,delta,length,cfg);
j = 0;
if isok
    for i = 1:size(pth,2)
        %ToDO：碰撞检测
        tvec = [px,py,pth(i)];
        isCollision = VehicleCollisionCheck(tvec,obstpoint,cfg);
        %         isCollision = false;
        
        if isCollision
            isok = false;
            return
        else
            j=j+1;
            [isok,xidx,yidx,thidx] = CalcIdx(cfg,px,py,pth(i)); % 把路径末端点的实际坐标转换为栅格坐标
            if isok == false
                return
            else
                cost = wknode.cost;
                if D > 0 % 前进
                    cost = cost + cfg.FORW_COST*gres*sqrt(2); % 每条轨迹大概是2米的长度。这里乘以1.5是确保下一个末端状态肯定在另一个栅格中，不会还在一个栅格中！在地图栅格中子结点拓展。比如对角线长度是1.4，此时还是在同一个栅格中
                else % 后退
                    cost = cost + cfg.BACK_COST*gres*sqrt(2);
                end
                if D ~= wknode.D
                    cost = cost + cfg.SB_COST;
                end
                cost = cost + cfg.STEER_COST*abs(delta);
                cost = cost + cfg.STEER_CHANGE_COST * abs(delta-wknode.delta);
                
                [results2, flag2, result_num2] = ik(cfg.dhparams, px, py, pth(i)+pi+cfg.params.zeta,cfg.params);
                
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
                    [wknode.xidx,wknode.yidx,wknode.yawidx],cost); % tnode是路径的末端点，cost为到当前状态到此路径末端状态的成本
            end
        end
    end
end
end
%%
function [x,y,theta,flag] = MapDynamic(x,y,theta,D,delta,L,cfg)
% 根据当前位姿和输入,计算下一位置的位姿
x = x+D*L*cos(theta+delta); % 运动学公式： x_dot = v_x * cos(theta); x_dot * t = v_x * t * cos(theta),在采样时间t内,则有x = x + v_x * t * cos(theta)，其中v_x * t=D
y = y+D*L*sin(theta+delta); % 运动学公式
[isok,xidx,yidx,~] = CalcIdx(cfg,x,y);
[theta,flag] = MapTheta(xidx, yidx, cfg, theta);
if flag
    theta = wrapToPi(theta);
end
end
%%
function [Theta,flag]= MapTheta(xidx, yidx, cfg, theta)
% 基于当前角度theta计算符合MAP的下一个角度（多个）并且要小于cfg.MAXTHETA
MAP = cfg.Map;
flag = 0;
% xidx = size(MAP,1)-xidx+1;
% yidx = size(MAP,2)-yidx+1;
Theta = [];
for i = 1:size(MAP,3)
    if MAP(xidx,yidx,i)==1
        if abs(-pi + wrapToPi(cfg.YAW_GRID_RESOLUTION_SEARCH*(i-1)) + pi - theta)<= cfg.MAXTHETA
            %             Theta = [Theta, -pi + cfg.YAW_GRID_RESOLUTION_SEARCH*(i-1) + pi];
            Theta = [Theta, -pi + wrapToPi(cfg.YAW_GRID_RESOLUTION_SEARCH*(i-1)) + pi];
            flag = 1;
        end
        
    end
end
end
%%
function [isok,xidx,yidx,thidx] = CalcIdx(cfg,x,y,theta)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 把位姿(x,y,theta)转换为grid上的栅格索引,如果不符合实际则isok=false
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if nargin < 4
    thidx = Inf;
else
    yawres = cfg.YAW_GRID_RESOLUTION; % 栅格地图的分辨率 [rad]
    theta = wrapToPi(theta); % 控制theta范围在[-pi,pi]区间
    thidx = ceil((theta-cfg.MINYAW)/yawres);
end

gres = cfg.XY_GRID_RESOLUTION; % 栅格地图的分辨率 [m]
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
%判断节点是否在open列表中，在open中，isopenFlag = 1,不在open中，isopenFlag = 0 .并反回索引号
isopenFlag = 0;
Id = 0;

%如果open列表为空，则不在open列表中
if  isempty(Open)
    isopenFlag = 0;
    
else %open列表不为空时
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