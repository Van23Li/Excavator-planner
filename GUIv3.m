function varargout = GUIv3(varargin)
% GUIV3 MATLAB code for GUIv3.fig
%      GUIV3, by itself, creates a new GUIV3 or raises the existing
%      singleton*.
%
%      H = GUIV3 returns the handle to a new GUIV3 or the handle to
%      the existing singleton*.
%
%      GUIV3('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUIV3.M with the given input arguments.
%
%      GUIV3('Property','Value',...) creates a new GUIV3 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before GUIv3_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to GUIv3_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help GUIv3

% Last Modified by GUIDE v2.5 17-Dec-2021 15:12:54

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @GUIv3_OpeningFcn, ...
    'gui_OutputFcn',  @GUIv3_OutputFcn, ...
    'gui_LayoutFcn',  [] , ...
    'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before GUIv3 is made visible.
function GUIv3_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to GUIv3 (see VARARGIN)

% Choose default command line output for GUIv3
handles.output = hObject;
global flag params dhparams params_c Configure
flag=0;%xzk:用于记录第几次按下鼠标，0为初始，1为选择了起点位姿，2为选择了终点位姿
params = parameters();
params.phi_0 = -2.5;
params_c = parameters_cal(params);
syms a2 a3 a4
dhparams_sym = [0	0	0	0;
    a2	0	0	0;
    a3	0	0	0;
    a4	0	0	0;];
dhparams = double(subs(dhparams_sym,[a2,a3,a4],[params_c.l_CF/1000,params_c.l_FQ/1000,params_c.l_QV/1000]));
%%
%xzk:初始化加载读图，这段主要是graph_construct.m
axes(handles.axes1);
load MAP_03.mat
% load ObstMap_03.mat
MAP = cat(3,MAP,MAP(:,:,1));
step = 0.3;%xzk:设置格子分辨率
step_phi = 0.1 * pi;
i_int = 0;

%画矩形
for i = -0.5 : step : 10-step
    rectangle('Position',[i -9 step 15],'EdgeColor','y');
end
for j = -9 : step : 6-step
    rectangle('Position',[-0.5 j 10.5 step],'EdgeColor','y');
end
rectangle('Position',[4.5 -3.5 2 2]);
axis([-0.5 10  -9 6]);
hold on

for i = -0.5 : step : 10-step
    i_int = i_int + 1;
    x = i + step/2;
    j_int = 0;
    for j = -9 : step : 6-step
        j_int = j_int + 1;
        y = j + step/2;
        fprintf(['i=', num2str(i_int), ' in total ', num2str((10-step+0.5)/step),'; j=', num2str(j_int), ' in total ', num2str((15-step)/step),'\n' ])
        
        %初始化变量
        phi_int = 0;
        phi_int_pre = 1;
        k = 0;
        k_len = 0;
        sector_angle = [];
        sector_length = [];
        for phi = -pi : step_phi : pi
            phi_int = phi_int + 1;
            if MAP(i_int,j_int,phi_int) == 1
                %ToDo: 变长度
                len_lambda = step/2;
                if phi_int - phi_int_pre == 1
                    if k == 0
                        k=k+1;
                        k_len = k_len + 1;
                        sector_angle(k,1) = (phi+pi);
                        sector_length(k,1) = len_lambda;
                    end
                    sector_angle(k,2) = (phi+pi);
                    sector_length(k,2) = len_lambda;
                    phi_int_pre = phi_int;
                else
                    k = k + 1;
                    k_len = k_len + 1;
                    sector_angle(k,1) = (phi+pi);
                    sector_angle(k,2) = (phi+pi);
                    sector_length(k,1) = len_lambda;
                    sector_length(k,2) = len_lambda;
                    %                         sector_length(k,k_len+1) = len_lambda;
                    phi_int_pre = phi_int;
                end
            end
        end
        %%%%%%%%%%%%%%%%一起画%%%%%%%%%%%%%%%%%%%%%%%%%
        if ~isempty(sector_angle)
            for num = 1 : size(sector_angle,1)
                %                 sector(x,y,sector_angle(num,1)+pi,sector_angle(num,2)+pi,step/2);
                sector_len(x,y,sector_angle(num,1),sector_angle(num,2),sector_length(num,:),step_phi);
                hold on
                axis equal
                axis([-0.5 10  -9 6]);
            end
        end
        
    end
end
%%
%xzk:初始化configure,这段主要是EntryPoint.m
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

for i = 4.5 : 0.1 : 6.5
    ObstList(end+1,:) = [i,-3.5];
end
for i = 4.5 : 0.1 : 6.5
    ObstList(end+1,:) = [i,1.5];
end
for i = -3.5 : 0.1 : 1.5
    ObstList(end+1,:) = [4.5,i];
end
for i = -3.5 : 0.1 : 1.5
    ObstList(end+1,:) = [6.5,i];
end

ObstLine = []; % Park lot line for collision check
% tLine = [-0.5, -9 , 10, -9]; %start_x start_y end_x end_y
% ObstLine(end+1,:) = tLine;
% tLine = [10, -9,  10, 6]; %start_x start_y end_x end_y
% ObstLine(end+1,:) = tLine;
% tLine = [10, 6 , -0.5, 6]; %start_x start_y end_x end_y
% ObstLine(end+1,:) = tLine;
% tLine = [-0.5, 6 , -0.5, -9]; %start_x start_y end_x end_y
% ObstLine(end+1,:) = tLine;

tLine = [4.5, -3.5 , 6.5, -3.5]; %start_x start_y end_x end_y
ObstLine(end+1,:) = tLine;
tLine = [6.5, -3.5,  6.5, -1.5]; %start_x start_y end_x end_y
ObstLine(end+1,:) = tLine;
tLine = [6.5, -1.5 , 4.5, -1.5]; %start_x start_y end_x end_y
ObstLine(end+1,:) = tLine;
tLine = [4.5, -1.5 , 4.5, -3.5]; %start_x start_y end_x end_y
ObstLine(end+1,:) = tLine;

% Configure.MAX_STEER = 0.6; % [rad] maximum steering angle

%Map
Configure.Map = MAP;
Configure.TolLen = 0.4; %距离误差
Configure.TolAng = 0.6*0.1*pi; %角度误差
Configure.AStarStyle = 2;   %A*在二维上运行还是三维上运行
Configure.ANGLE_COST = 1;
Configure.AStar_COST = 1;
Configure.Node_COST = 1;

% ObstList and ObstLine
Configure.ObstList = ObstList;
Configure.ObstLine = ObstLine;

% Motion resolution define
Configure.MOTION_RESOLUTION = 1; % [m] path interporate resolution
Configure.N_STEER = 5; % number of steer command
Configure.EXTEND_AREA = 0; % [m] map extend length
Configure.XY_GRID_RESOLUTION = 0.3; % [m] 与MAP.mat对应
Configure.YAW_GRID_RESOLUTION = 0.1*pi; % [rad] 与MAP.mat对应

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

% % 使用完整约束有障碍情况下用A*搜索的最短路径最为hybrid A*的启发值
% ObstMap = GridAStar(Configure,End);
% % save ObstMap_03 ObstMap
% % load ObstMap_03.mat
% % m=mesh(ObstMap);
% % m.FaceLighting = 'phong';
% % axis equal;
% Configure.ObstMap = ObstMap;

% Update handles structure
%%
%xzk：画初始挖掘机
global hbc hbd hdf hef hqf hqg heg hmn hmk hqk hqv hzv hzk hed hab hgm
[results1, flag1, result_num1] = ik(dhparams, 7, 3, 0.6*pi+pi+params.zeta,params);%xzk:默认位姿7,3,0.6pi
[C, B, D, F, Q, V, E, G, K, A, M, N] = convert(0, results1(2,1), results1(2,2), results1(2,3), params_c);
[hbc, hbd, hdf, hef ,hqf ,hqg ,heg ,hmn, hmk, hqk, hqv, hzv, hzk, hed ,hab ,hgm]=DrawPoints_ZK(C, B, D, F, Q, V, E, G, K, A, M, N, params);

%%
%xzk:初始化axes2 axes3
axes(handles.axes2);
axis equal
axis([-0.5 10  -9 6]);

axes(handles.axes3);
axis equal
axis([-0.5 10  -9 6]);

%%
axes(handles.detail_a1);
ylabel('angle(^o)')
xlabel('step')

axes(handles.detail_a2);
ylabel('angle(^o)')
xlabel('step')

axes(handles.detail_a3);
ylabel('angle(^o)')
xlabel('step')
%%

guidata(hObject, handles);

% UIWAIT makes GUIv3 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = GUIv3_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function edit1_x_Callback(hObject, eventdata, handles)
% hObject    handle to edit1_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1_x as text
%        str2double(get(hObject,'String')) returns contents of edit1_x as a double


% --- Executes during object creation, after setting all properties.
function edit1_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit1_y_Callback(hObject, eventdata, handles)
% hObject    handle to edit1_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1_y as text
%        str2double(get(hObject,'String')) returns contents of edit1_y as a double


% --- Executes during object creation, after setting all properties.
function edit1_y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit1_theta_Callback(hObject, eventdata, handles)
% hObject    handle to edit1_theta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1_theta as text
%        str2double(get(hObject,'String')) returns contents of edit1_theta as a double


% --- Executes during object creation, after setting all properties.
function edit1_theta_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1_theta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_x_Callback(hObject, eventdata, handles)
% hObject    handle to edit2_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2_x as text
%        str2double(get(hObject,'String')) returns contents of edit2_x as a double


% --- Executes during object creation, after setting all properties.
function edit2_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_y_Callback(hObject, eventdata, handles)
% hObject    handle to edit2_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2_y as text
%        str2double(get(hObject,'String')) returns contents of edit2_y as a double


% --- Executes during object creation, after setting all properties.
function edit2_y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_theta_Callback(hObject, eventdata, handles)
% hObject    handle to edit2_theta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2_theta as text
%        str2double(get(hObject,'String')) returns contents of edit2_theta as a double


% --- Executes during object creation, after setting all properties.
function edit2_theta_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2_theta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on mouse press over figure background, over a disabled or
% --- inactive control, or over an axes background.
function figure1_WindowButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global x1 y1 theta1 x2 y2 theta2 flag
switch flag %xzk:用于记录第几次按下鼠标，0为初始，1为选择了起点位姿，2为选择了终点位姿
    case 0
        if strcmp(get(gcf,'SelectionType'),'normal')
            pos1=get(handles.axes1,'CurrentPoint');%xzk:记录按下鼠标的位置
            x1=pos1(1,1);
            y1=pos1(1,2);
            
        end
    case 1
        if strcmp(get(gcf,'SelectionType'),'normal')
            pos1=get(handles.axes1,'CurrentPoint');
            x2=pos1(1,1);
            y2=pos1(1,2);
            
        end
        
        
end

% --- Executes on mouse press over figure background, over a disabled or
% --- inactive control, or over an axes background.
function figure1_WindowButtonUpFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global x1 y1 theta1 x2 y2 theta2 flag h1 h2 Configure
switch flag %xzk:用于记录第几次按下鼠标，0为初始，1为选择了起点位姿，2为选择了终点位姿
    case 0
        if strcmp(get(gcf,'SelectionType'),'normal')
            pos2=get(handles.axes1,'CurrentPoint');%xzk:记录松开鼠标的位置
            theta1=atan2(y1-pos2(1,2),x1-pos2(1,1));
            
            %             %xzk:这两步把xy量化为0.5的倍数
            %             x1=floor(x1/0.5)*0.5;
            %             y1=floor(y1/0.5)*0.5;
            %
            %             %xzk:这一步把theta量化为pi/10的倍数
            %             theta1=floor(theta1/(pi/10))*pi/10;
            
            %xzk:在界面上显示出位姿值
            set(handles.edit1_x,'String',x1);
            set(handles.edit1_y,'String',y1);
            set(handles.edit1_theta,'String',theta1);
            
            axes(handles.axes1);
            h1=quiver(x1,y1,-cos(theta1),-sin(theta1),'-g','LineWidth',2,'MaxHeadSize',2);%xzk:画起始位姿箭头
            flag=flag+1;
            
        end
    case 1
        if strcmp(get(gcf,'SelectionType'),'normal')
            pos2=get(handles.axes1,'CurrentPoint');
            theta2=atan2(y2-pos2(1,2),x2-pos2(1,1));
            
            %              %xzk:这两步把xy量化为0.5的倍数
            %             x2=floor(x2/0.5)*0.5;
            %             y2=floor(y2/0.5)*0.5;
            %
            %             %xzk:这一步把theta量化为pi/10的倍数
            %             theta2=floor(theta2/(pi/10))*pi/10;
            
            set(handles.edit2_x,'String',x2);
            set(handles.edit2_y,'String',y2);
            set(handles.edit2_theta,'String',theta2);
%             Configure.End=[x2,y2,theta2];
            Configure.End = [2, -1, -pi/2];
            
            axes(handles.axes1);
            h2=quiver(x2,y2,-cos(theta2),-sin(theta2),'-y','LineWidth',2,'MaxHeadSize',2);%xzk:画终止位姿箭头
            flag=flag+1;
        end
        
end


% --- Executes on button press in Startbutton.
function Startbutton_Callback(hObject, eventdata, handles) %xzk:start按钮
% hObject    handle to Startbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global params dhparams params_c x1 y1 theta1 x2 y2 theta2 flag h1 h2 Configure nodes
if flag ==2 %xzk:用于记录第几次按下鼠标，0为初始，1为选择了起点位姿，2为选择了终点位姿
    phi1 = theta1+pi;
    phi2 = theta2+pi;
    [results1, flag1, result_num1] = ik(dhparams, x1, y1, phi1+params.zeta,params);
    [results2, flag2, result_num2] = ik(dhparams, x2, y2, phi2+params.zeta,params);
    if flag1 == 1
        [flag1, flag1_p, flag1_n] = exam_reach(results1, params_c);
    end
    if flag2 == 1
        [flag2, flag2_p, flag2_n] = exam_reach(results2, params_c);
    end
    if flag1==1&&flag2==1
        %xzk:若起终点位姿均可行，调用主函数
        set(handles.textedit,'String','finding Astar......');
        pause(0.1);
        % 使用完整约束有障碍情况下用A*搜索的最短路径最为hybrid A*的启发值
%         End = [x2, y2, theta2];
        End = [2, -1, -pi/2];
%         ObstMap = GridAStar(Configure,End);
        % save ObstMap_03 ObstMap
        load ObstMap_03.mat
        % m=mesh(ObstMap);
        % m.FaceLighting = 'phong';
        % axis equal;
        Configure.ObstMap = ObstMap;
        set(handles.textedit,'String','finding path......');
        pause(0.1);
        nodes = VanAStar([x1 y1 theta1],[x2 y2 theta2],Configure,handles);
        delete(h1);
        delete(h2);
        flag=0;
    else
        set(handles.textedit,'String','invalid pose,please set again');
        delete(h1);
        delete(h2);
        flag=0;
    end
end


function textedit_Callback(hObject, eventdata, handles)
% hObject    handle to textedit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of textedit as text
%        str2double(get(hObject,'String')) returns contents of textedit as a double


% --- Executes during object creation, after setting all properties.
function textedit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to textedit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Resetbutton.
function Resetbutton_Callback(hObject, eventdata, handles) %xzk：reset按钮，重置起点和终点位姿，初始化坐标图1、2、3，
% hObject    handle to Resetbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global h1 h2 flag nodes hbc hbd hdf hef hqf hqg heg hmn hmk hqk hqv hzv hzk hed hab hgm params  params_c dhparams
axes(handles.axes2);
cla reset;
axes(handles.axes3);
cla reset;
axes(handles.detail_a1);
cla reset;
ylabel('angle(^o)')
xlabel('step')
axes(handles.detail_a2);
cla reset;
ylabel('angle(^o)')
xlabel('step')
axes(handles.detail_a3);
cla reset;
ylabel('angle(^o)')
xlabel('step')
axes(handles.axes2);
axis equal
axis([-0.5 10  -9 6]);
axes(handles.axes3);
axis equal
axis([-0.5 10  -9 6]);
delete(h1);
delete(h2);
set(handles.edit1_x,'String',[]);
set(handles.edit1_y,'String',[]);
set(handles.edit1_theta,'String',[]);
set(handles.edit2_x,'String',[]);
set(handles.edit2_y,'String',[]);
set(handles.edit2_theta,'String',[]);
nodes=[];
delete(hbc);delete(hbd);delete(hdf);delete(hef);delete(hqf);delete(hqg);delete(heg); delete(hmn);delete(hmk);delete(hqk); delete(hqv);delete(hzv); delete(hzk);delete(hed); delete(hab); delete(hgm);
[results1, flag1, result_num1] = ik(dhparams, 7, 3, 0.6*pi+pi+params.zeta,params);
[C, B, D, F, Q, V, E, G, K, A, M, N] = convert(0, results1(2,1), results1(2,2), results1(2,3), params_c);
axes(handles.axes1);
[hbc, hbd, hdf, hef ,hqf ,hqg ,heg ,hmn, hmk, hqk, hqv, hzv, hzk, hed ,hab ,hgm]=DrawPoints_ZK(C, B, D, F, Q, V, E, G, K, A, M, N, params);
flag=0;


% --- Executes on button press in animationbutton.
function animationbutton_Callback(hObject, eventdata, handles)
% hObject    handle to animationbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global nodes params  params_c dhparams hbc hbd hdf hef hqf hqg heg hmn hmk hqk hqv hzv hzk hed hab hgm
y1=[];
y2=[];
y3=[];

if ~isempty(nodes)
    
    for i=length(nodes):-1:1
        x = nodes(i).x;
        y = nodes(i).y;
        phi = nodes(i).theta+pi;
        [results1, flag1, result_num1] = ik(dhparams, x, y, phi+params.zeta,params);
        
        if flag1 == 1
            [flag1, flag1_p, flag1_n] = exam_reach(results1, params_c);
        end
        
        if flag1 == 1
            axes(handles.axes1);
            [C, B, D, F, Q, V, E, G, K, A, M, N] = convert(0, results1(2,1), results1(2,2), results1(2,3), params_c);
            delete(hbc);delete(hbd);delete(hdf);delete(hef);delete(hqf);delete(hqg);delete(heg); delete(hmn);delete(hmk);delete(hqk); delete(hqv);delete(hzv); delete(hzk);delete(hed); delete(hab); delete(hgm);
            %xzk:把上一帧的句柄删掉再画下一帧
            [hbc, hbd, hdf, hef ,hqf ,hqg ,heg ,hmn, hmk, hqk, hqv, hzv, hzk, hed ,hab ,hgm]=DrawPoints_ZK(C, B, D, F, Q, V, E, G, K, A, M, N, params);
            
            axes(handles.detail_a1);
            cla reset;
            ta=atan2(B(2),B(1));
            ta=ta*180/pi;
            y1=[y1 ta];
            if length(y1)>1
                n=1:length(y1);
                nn=1:0.1:length(y1);
                yy1=interp1(n,y1,nn,'spline');
                plot(nn,yy1);
                ylabel('angle(^o)')
                xlabel('step')
            end
            
            axes(handles.detail_a2);
            cla reset;
            t1=atan2(Q(2)-F(2),Q(1)-F(1));
            t2=atan2(D(2)-F(2),D(1)-F(1));
            if t1<0
                t1=t1+2*pi;
            end
            if t2<0
                t2=t2+2*pi;
            end
            tb=(t1-t2)*180/pi;
            y2=[y2 tb];
            if length(y2)>1
                n=0:length(y2)-1;
                nn=0:0.1:length(y2)-1;
                yy2=interp1(n,y2,nn,'spline');
                plot(nn,yy2);
                ylabel('angle(^o)')
                xlabel('step')
            end
            
            axes(handles.detail_a3);
            cla reset;
            t1=atan2(V(2)-Q(2),V(1)-Q(1));
            t2=atan2(F(2)-Q(2),F(1)-Q(1));
            if t1<0
                t1=t1+2*pi;
            end
            if t2<0
                t2=t2+2*pi;
            end
            tc=(t1-t2)*180/pi;
            y3=[y3 tc];
            if length(y3)>1
                n=0:length(y3)-1;
                nn=0:0.1:length(y3)-1;
                yy3=interp1(n,y3,nn,'spline');
                plot(nn,yy3);
                ylabel('angle(^o)')
                xlabel('step')
            end
            pause(0.3);%xzk:两帧间隔
            
        end
    end
end

