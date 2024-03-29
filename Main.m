%%
clear all
close all
addpath('Planner');
addpath('Visual');
%%
Start = [7.8, -4.2, pi/2];
End = [2, -1, -pi/2];
Display = 0;
%%
Configure.params = parameters();
Configure.params.phi_0 = 0;
Configure.params_c = parameters_cal(Configure.params);
syms a2 a3 a4
dhparams_sym = [0	0	0	0;
    a2	0	0	0;
    a3	0	0	0;
    a4	0	0	0;];
Configure.dhparams = double(subs(dhparams_sym,[a2,a3,a4],[Configure.params_c.l_CF/1000,Configure.params_c.l_FQ/1000,Configure.params_c.l_QV/1000]));

%%
ObstPoint = [];
ObstPoint(1,:) = [5, -2];
ObstPoint(2,:) = [6, -3];
Configure.ObstPoint = ObstPoint;
%%
ObstList = []; % Obstacle point list
for i = -0.5 : 0.1 : 10
    ObstList(end+1,:) = [i,-9];
end
for i = -0.5 : 0.1 : 10
    ObstList(end+1,:) = [i,6];
end
for i = -9 : 0.1 : 6
    ObstList(end+1,:) = [-0.5,i];
end
for i = -9 : 0.1 : 6
    ObstList(end+1,:) = [10,i];
end

for i = ObstPoint(1,1) : 0.1 : ObstPoint(2,1)
    ObstList(end+1,:) = [i,ObstPoint(2,2)];
end
for i = ObstPoint(1,1) : 0.1 : ObstPoint(2,1)
    ObstList(end+1,:) = [i,ObstPoint(1,2)];
end
for i = ObstPoint(2,2) : 0.1 : ObstPoint(1,2)
    ObstList(end+1,:) = [5.5,i];
end
for i = ObstPoint(2,2) : 0.1 : ObstPoint(1,2)
    ObstList(end+1,:) = [ObstPoint(1,2),i];
end
%%
ObstLine = []; % Park lot line for collision check
tLine = [9.5, 5.5 , 10, 5.5]; %start_x start_y end_x end_y
ObstLine(end+1,:) = tLine;
tLine = [10, 5.5,  10, 6]; %start_x start_y end_x end_y
ObstLine(end+1,:) = tLine;
tLine = [10, 6 , 9.5, 6]; %start_x start_y end_x end_y
ObstLine(end+1,:) = tLine;
tLine = [9.5, 6 , 9.5, 2.5]; %start_x start_y end_x end_y
ObstLine(end+1,:) = tLine;
%%
% Configure.MAX_STEER = 0.6; % [rad] maximum steering angle

% load MAP_lambda1.mat
% load MAP_lambda2.mat
% load MAP_lambda3.mat
Configure.Drive_COST = 50;
% Configure.MAP_lambda1 = MAP_lambda1;
% Configure.MAP_lambda2 = MAP_lambda2;
% Configure.MAP_lambda3 = MAP_lambda3;

%Map
load('MAP_03.mat');
Configure.Map = MAP;
Configure.TolLen = 0.4; %距离误差
Configure.TolAng = 0.6*0.1*pi; %角度误差
Configure.AStarStyle = 2;   %A*在二维上运行还是三维上运行
Configure.ANGLE_COST = 0.2;
Configure.AStar_COST = 1;
Configure.Node_COST = 1;

% ObstList and ObstLine
Configure.ObstLine = ObstLine;
Configure.ObstPoint = ObstPoint;

% Motion resolution define
Configure.MOTION_RESOLUTION = 1; % [m] path interporate resolution
Configure.N_STEER = 5; % number of steer command 5
Configure.EXTEND_AREA = 0; % [m] map extend length
Configure.XY_GRID_RESOLUTION = 0.3; % 栅格地图的分辨率 [m] 与MAP.mat对应
Configure.XY_GRID_RESOLUTION_SEARCH = 0.3;
Configure.YAW_GRID_RESOLUTION = 0.1*pi; % 栅格地图的分辨率 [rad] 与MAP.mat对应
Configure.YAW_GRID_RESOLUTION_SEARCH = 0.1*pi; % [rad] 与MAP.mat对应

% Grid bound
Configure.MINX = -0.5-Configure.EXTEND_AREA;
Configure.MAXX = 10+Configure.EXTEND_AREA;
Configure.MINY = -9-Configure.EXTEND_AREA;
Configure.MAXY = 6+Configure.EXTEND_AREA;
Configure.MAXTHETA = 0.1*pi;
Configure.MINYAW = -pi;
Configure.MAXYAW = pi;

% Cost related define
Configure.SB_COST = 0; % switch back penalty cost
Configure.FORW_COST = 1; % forward penalty cost
Configure.BACK_COST = 1; % backward penalty cost
Configure.STEER_CHANGE_COST = 0; % steer angle change penalty cost
Configure.STEER_COST = 0; % steer angle change penalty cost
Configure.H_COST = 10; % Heuristic cost

Configure.End = End;

if ~isempty('ObstMap_03.mat')
    load ObstMap_03.mat
else
    ObstMap = GridAStar(Configure,End);
    save ObstMap_03 ObstMap
end
Configure.ObstMap = ObstMap;

nodes = VanAStar(Start,End,Display,Configure);
save nodes nodes

if isempty(nodes)
    disp("Failed to find path!")
else
    figure(3)
    Animation(nodes,Configure,3);
end