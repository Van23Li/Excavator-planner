function [C, B, D, F, Q, V, E, G, K, A, M, N] = convert_bucket(phi_0, phi_1, phi_2, phi_3, params_c)
%输入：关节空间的参数
%输出：挖掘机上各关键点在世界坐标系下的坐标
%%
q_3 = pi + phi_3;
% 参数赋值
% cc = params_c.cc;
% bb = params_c.bb;
% aa = params_c.aa;
cc = 0;
bb = 0;
aa = 0;

l_BC = params_c.l_BC;
D_BCF = params_c.D_BCF;
l_CF = params_c.l_CF;
l_FQ = params_c.l_FQ;
l_3 = params_c.l_QV;
l_CD = params_c.l_CD;
D_DCF = params_c.D_DCF;
l_EF = params_c.l_EF;
D_EFQ = params_c.D_EFQ;
l_FG = params_c.l_FG;
D_GFQ = params_c.D_GFQ;
l_KQ = params_c.l_KQ;
D_KQV = params_c.D_KQV;
A_x = params_c.A_x;
A_y = params_c.A_y;
l_FN = params_c.l_FN;
D_FQN = params_c.D_FQN;
l_NQ = params_c.l_NQ;
l_MN = params_c.l_MN;
%% 回转平台的运动分析
R_z0phi0 = [cos(phi_0), -sin(phi_0), 0;...
    sin(phi_0),  cos(phi_0), 0;...
    0      ,        0,          1]; % 转台相对于z0轴的旋转矩阵
xyz_0C = [0; cc; bb];
XYZ_C = R_z0phi0 * xyz_0C + [0; 0; aa];
C = XYZ_C(2:3);

xyz_0A = [0; A_x; A_y];
XYZ_A = R_z0phi0 * xyz_0A + [0; 0; aa];
A = XYZ_A(2:3);

R_x1phi1 = [1, 0, 0;...
    0,  cos(phi_1), -sin(phi_1);...
    0,   sin(phi_1), cos(phi_1)]; % 动臂上各铰接点绕x1轴的旋转矩阵
xyz_1B = [0; l_BC*cos(D_BCF); l_BC*sin(D_BCF)];
XYZ_B = R_z0phi0 * (xyz_0C + R_x1phi1 * xyz_1B) + [0; 0; aa];
B = XYZ_B(2:3);

xyz_1D = [0; l_CD*cos(D_DCF); l_CD*sin(D_DCF)];
XYZ_D = R_z0phi0 * (xyz_0C + R_x1phi1 * xyz_1D) + [0; 0; aa];
D = XYZ_D(2:3);

xyz_1F = [0; l_CF; 0];
XYZ_F = R_z0phi0 * (xyz_0C + R_x1phi1 * xyz_1F) + [0; 0; aa];
F = XYZ_F(2:3);

R_x2phi2 = [1, 0, 0;...
    0,  cos(phi_2), -sin(phi_2);...
    0,   sin(phi_2), cos(phi_2)]; % 斗杆上各铰接点绕x2轴的旋转矩阵
xyz_2Q = [0; l_FQ; 0];
XYZ_Q = R_z0phi0 * (xyz_0C + R_x1phi1 * (xyz_1F + R_x2phi2 * xyz_2Q)) + [0; 0; aa];
Q = XYZ_Q(2:3);

xyz_2N = [0; l_FQ - l_NQ * cos(D_FQN); l_NQ * sin(D_FQN)];
XYZ_N = R_z0phi0 * (xyz_0C + R_x1phi1 * (xyz_1F + R_x2phi2 * xyz_2N)) + [0; 0; aa];
N = XYZ_N(2:3);

l_KN = sqrt(params_c.l_NQ^2 + params_c.l_KQ^2 ...
    - 2 * params_c.l_NQ * params_c.l_KQ * cos(2 * pi - params_c.D_KQV - q_3 - params_c.D_FQN));
if q_3 <= params_c.D_KVQ + params_c.D_VKQ
    D_QNM = -asin(params_c.l_KQ / l_KN * sin(2* pi - params_c.D_KQV - q_3 - params_c.D_FQN))...
        + acos((params_c.l_MN^2 + l_KN^2 - params_c.l_MK^2)...
        /(2 * params_c.l_MN * l_KN));
elseif q_3 > params_c.D_KVQ + params_c.D_VKQ
    D_QNM = asin(params_c.l_KQ / l_KN * sin(2* pi - params_c.D_KQV - q_3 - params_c.D_FQN))...
        + acos((params_c.l_MN^2 + l_KN^2 - params_c.l_MK^2)...
        /(2 * params_c.l_MN * l_KN));
end
xyz_2M = [0;l_FQ - l_NQ * cos(D_FQN) + l_MN * cos(D_QNM); l_NQ * sin(D_FQN) + l_MN * sin(D_QNM)];
XYZ_M = R_z0phi0 * (xyz_0C + R_x1phi1 * (xyz_1F + R_x2phi2 * xyz_2M)) + [0; 0; aa];
M = XYZ_M(2:3);

xyz_2E = [0; l_EF * cos(D_EFQ); l_EF * sin(D_EFQ)];
XYZ_E = R_z0phi0 * (xyz_0C + R_x1phi1 * (xyz_1F + R_x2phi2 * xyz_2E)) + [0; 0; aa];
E = XYZ_E(2:3);

xyz_2G = [0; l_FG * cos(D_GFQ); l_FG * sin(D_GFQ)];
XYZ_G = R_z0phi0 * (xyz_0C + R_x1phi1 * (xyz_1F + R_x2phi2 * xyz_2G)) + [0; 0; aa];
G = XYZ_G(2:3);

R_x3phi3 = [1, 0, 0;...
    0,  cos(phi_3), -sin(phi_3);...
    0,   sin(phi_3), cos(phi_3)]; % 铲斗上各铰接点绕x3轴的旋转矩阵
xyz_3V = [0; l_3; 0];
XYZ_V = R_z0phi0 * (xyz_0C + R_x1phi1 * (xyz_1F + R_x2phi2 * (xyz_2Q + R_x3phi3 * xyz_3V))) + [0; 0; aa];
V = XYZ_V(2:3);

xyz_3K = [0; l_KQ * cos(D_KQV); l_KQ * sin(D_KQV)];
XYZ_K = R_z0phi0 * (xyz_0C + R_x1phi1 * (xyz_1F + R_x2phi2 * (xyz_2Q + R_x3phi3 * xyz_3K))) + [0; 0; aa];
K = XYZ_K(2:3);
end