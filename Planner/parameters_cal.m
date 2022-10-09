function params_c = parameters_cal(params)
aa = -params.y + params.C_y;
bb = -params.C_y;
cc = -params.C_x;
phi_0 = params.phi_0;
l_AC = sqrt(params.A_x^2 + params.A_y^2);
D_ACx = abs(atan(params.A_y/params.A_x));

l_BC = params.BC;
l_CF = params.CF;
l_CD = params.CD;
l_DF = params.DF;
l_EF = params.EF;
D_CBF = acos((params.BC^2 + params.BF^2 - params.CF^2)/(2 * params.BC * params.BF));
D_BCF = acos((params.BC^2 + params.CF^2 - params.BF^2) / (2 * params.BC * params.CF));
D_CFB = pi - D_CBF - D_BCF;
D_CFD = acos((params.DF^2 + params.CF^2 - params.CD^2)/ (2 * params.DF * params.CF)); 
D_DCF = acos((params.CD^2 + params.CF^2 - params.DF^2)/(2 * params.CD * params.CF));

l_EF = params.EF;
l_FG = params.FG;
l_EG = params.EG;
l_GQ = params.GQ;
l_FQ = params.FQ;
l_GN = params.GN;
l_NQ = params.NQ;
D_EFQ = acos((params.FQ^2 + params.FG^2 - params.GQ^2)/ (2 * params.FQ * params.FG))...
    +acos((params.FG^2 + params.EF^2 - params.EG^2)/ (2 * params.FG * params.EF));
D_GFQ = acos((params.FQ^2 + params.FG^2 - params.GQ^2)/ (2 * params.FQ * params.FG)); 
D_FQG = acos((params.FQ^2 + params.GQ^2 - params.FG^2)/ (2 * params.FQ * params.GQ));
D_NQG = acos((params.NQ^2 + params.GQ^2 - params.GN^2)/ (2 * params.NQ * params.GQ));

l_KQ = params.KQ;
l_QV = params.QV;
l_KV = params.KV;
l_MN = params.MN;
l_MK = params.MK;
D_KVQ = acos((params.KV^2 + params.QV^2 - params.KQ^2)/(2 * params.KV * params.QV));
D_VKQ = acos((params.KV^2 + params.KQ^2 - params.QV^2)/(2 * params.KV * params.KQ));
D_KQV = acos((params.KQ^2 + params.QV^2 - params.KV^2)/(2 * params.KQ * params.QV));

%% 
l_AB_min = params.AB_min;
l_AB_max = params.AB_max;

l_FQ_min = params.DE_min;
l_FQ_max = params.DE_max;

l_GM_min = params.GM_min;
l_GM_max = params.GM_max;

%%
if (l_AC + l_AB_min)>l_BC && abs(l_AC - l_BC)<l_AB_min && abs(l_BC - l_AB_min) < l_AC ...
        && (l_AC + l_BC)>l_AB_max && abs(l_AC - l_AB_max)<l_BC && abs(l_BC - l_AB_max)<l_AC
else
    over_all = 100;
    error('动臂机构存在干涉！')
    return
end

%%
phi_1_max = acos((l_AC^2 + l_BC^2 - l_AB_max^2)/ (2 * l_AC * l_BC)) - D_ACx - D_BCF;
phi_1_min = acos((l_AC^2 + l_BC^2 - l_AB_min^2)/ (2 * l_AC * l_BC)) - D_ACx - D_BCF; 

%% 
if (l_EF + l_FQ_min)>l_DF && abs(l_EF - l_DF)<l_FQ_min && abs(l_DF - l_FQ_min) < l_EF ...
        && (l_EF + l_DF)>l_FQ_max && abs(l_EF - l_FQ_max)<l_DF && abs(l_DF - l_FQ_max)<l_EF
else
    over_all = 100;
    error('斗杆机构存在干涉！')
    return
end

%% 斗杆摆角
phi_2_min = pi - D_CFD - D_EFQ - acos((l_DF^2 + l_EF^2 - l_FQ_max^2) / (2 * l_DF * l_EF));
phi_2_max = pi - D_CFD - D_EFQ - acos((l_DF^2 + l_EF^2 - l_FQ_min^2) / (2 * l_DF * l_EF));

%%
z_N_3 = params.NQ * sin(D_FQG - D_NQG);

D_MNG_min = acos((l_GN^2 + l_MN^2 - l_GM_min^2) / (2 * l_GN * l_MN));
D_MNG_max = acos((l_GN^2 + l_MN^2 - l_GM_max^2) / (2 * l_GN * l_MN));

l_FN = sqrt((l_FQ - sqrt(l_NQ^2 - z_N_3^2))^2 + z_N_3^2);
D_FNG =acos((l_FN^2 + l_GN^2 - l_FG^2) / (2 * l_FN * l_GN));
D_FNQ =acos((l_FN^2 + l_NQ^2 - l_FQ^2) / (2 * l_FN * l_NQ));
D_FQN =acos((l_FQ^2 + l_NQ^2 - l_FN^2) / (2 * l_FQ * l_NQ));

l_MQ_max = sqrt(l_NQ^2 + l_MN^2 - 2 * l_NQ * l_MN * cos(2*pi - D_MNG_min - D_FNG - D_FNQ));
l_MQ_min = sqrt(l_NQ^2 + l_MN^2 - 2 * l_NQ * l_MN * cos(2*pi - D_MNG_max - D_FNG - D_FNQ));
D_NQM_max = acos((l_NQ^2 + l_MQ_min^2 - l_MN^2) / (2 * l_NQ * l_MQ_min));
D_NQM_min = acos((l_NQ^2 + l_MQ_max^2 - l_MN^2) / (2 * l_NQ * l_MQ_max));
D_MQK_max = acos((l_KQ^2 + l_MQ_min^2 - l_MK^2) / (2 * l_KQ * l_MQ_min));
D_MQK_min = acos((l_KQ^2 + l_MQ_max^2 - l_MK^2) / (2 * l_KQ * l_MQ_max));
phi_3_max = -(D_FQN + D_NQM_min + D_MQK_min + D_KQV - pi);
phi_3_min = -(D_FQN + D_NQM_max + D_MQK_max + D_KQV - pi);

%%
params_c.phi_1_max = phi_1_max;
params_c.phi_1_min = phi_1_min;

params_c.phi_2_max = phi_2_max;
params_c.phi_2_min = phi_2_min;

params_c.phi_3_max = phi_3_max;
params_c.phi_3_min = phi_3_min;

params_c.aa = aa;
params_c.bb = bb;
params_c.cc = cc;

params_c.l_AC = l_AC;
params_c.l_BC = l_BC;
params_c.D_BCF = D_BCF;
params_c.D_ACx = D_ACx;

params_c.D_EFQ = D_EFQ;
params_c.D_CFD = D_CFD;
params_c.l_DF = l_DF;
params_c.l_EF = l_EF;

params_c.l_GN = l_GN;
params_c.l_MN = l_MN;
params_c.D_FNQ = D_FNQ;
params_c.D_FNG = D_FNG;
params_c.l_KQ = l_KQ;
params_c.l_MK = l_MK;
params_c.l_NQ = l_NQ;
params_c.l_MN = l_MN;
params_c.D_FQN = D_FQN;
params_c.D_KQV = D_KQV;
params_c.D_KVQ = D_KVQ;
params_c.D_VKQ = D_VKQ;

params_c.l_CF = l_CF;
params_c.l_FQ = l_FQ;
params_c.l_QV = l_QV;
params_c.D_ACx = D_ACx;
params_c.l_AC = l_AC;
params_c.l_CD = l_CD;
params_c.D_DCF = D_DCF;
params_c.l_FG = l_FG;
params_c.D_GFQ = D_GFQ;
params_c.A_x = params.A_x;
params_c.A_y = params.A_y;
params_c.l_FN = l_FN;
params_c.D_FQN = D_FQN;
end