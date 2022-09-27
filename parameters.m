function params = parameters()
% 工作装置尺寸
% 基准，动臂根铰点C为原点
params.A_x = 497.00;
params.A_y = -644.50;
params.C_x = -100.000;
params.C_y = -800.00;
params.y = -1902.00;

% 动臂
params.BC = 2508.00;
params.CF = 5700.00;
params.CD = 3379.60;
params.DF = 2807.70;
params.BF = 3601.20;

% 斗杆
params.EF = 850.00;
params.FG = 619.70;
params.EG = 1173.90;
params.GQ = 2656.00;
params.FQ = 2925.00;
params.GN = 2249.60;
params.NQ = 410.00;

% 铲斗
params.KQ = 458.20;
params.QV = 1469.80;
params.KV = 1605.10;
params.zeta = deg2rad([45]); %% 铲斗角度

% 连杆
params.MN = 640.00;
params.MK = 600.00;
%%
% 液压缸尺寸
params.AB_home = 2667.00;
params.DE_home = 2402.00;
params.GM_home = 1786.00;
params.AB_min = 1869.00;
params.AB_max = 3154.00;
params.DE_min = 2130.00;
params.DE_max = 3620.00;
params.GM_min = 1713.00;
params.GM_max = 2825.00;
end