%可视化逆解
%%
close all;
clear all;
params = parameters();
params.phi_0 = -2.5; % 转台绕z轴转动的角度

params_c = parameters_cal(params);

%%
syms a2 a3 a4
MAP = [];

%D-H参数
dhparams_sym = [0	0	0	0;
    a2	0	0	0;
    a3	0	0	0;
    a4	0	0	0;];
dhparams = double(subs(dhparams_sym,[a2,a3,a4],[params_c.l_CF/1000,params_c.l_FQ/1000,params_c.l_QV/1000]));
%%
%%%%%%%%%%%%%%%%%%%%%%%%%循环，构建graph
step = 0.3;     %x,y步长
step_phi = 0.1 * pi;        %角度步长
x = 6.5;
y = 1;
phi = pi/2-0.1*pi+pi;

%对两个对应的位姿进行逆解
[results1, flag1, result_num1] = ik(dhparams, x, y, phi+params.zeta,params);
[results2, flag2, result_num2] = ik(dhparams, x-step/2*cos(phi), y-step/2*sin(phi), phi+params.zeta,params);

%判断是否超过挖掘机限制
if flag1 == 1 && flag2 ==1
    [flag1, flag1_p, flag1_n] = exam_reach(results1, params_c);
    [flag2, flag2_p, flag2_n] = exam_reach(results2, params_c);
    
    if flag1 == 1 && flag2 == 1
        lambda1_p_1 = 0; lambda1_p_2 = 0; lambda1_p_3 = 0;
        lambda1_n_1 = 0; lambda1_n_2 = 0; lambda1_n_3 = 0;
        lambda2_p_1 = 0; lambda2_p_2 = 0; lambda2_p_3 = 0;
        lambda2_n_1 = 0; lambda2_n_2 = 0; lambda2_n_3 = 0;
        if flag1_p == 1
            [lambda1_p_1, lambda1_p_2, lambda1_p_3] = work2drive(results1(1,1), results1(1,2), results1(1,3), params_c);
        end
        if flag1_n == 1
            [lambda1_n_1, lambda1_n_2, lambda1_n_3] = work2drive(results1(2,1), results1(2,2), results1(2,3), params_c);
        end
        if flag2_p == 1
            [lambda2_p_1, lambda2_p_2, lambda2_p_3] = work2drive(results2(1,1), results2(1,2), results2(1,3), params_c);
        end
        if flag2_n == 1
            [lambda2_n_1, lambda2_n_2, lambda2_n_3] = work2drive(results2(2,1), results2(2,2), results2(2,3), params_c);
        end
%         lambda1_p_1 - lambda2_p_1;
%         lambda1_p_2 - lambda2_p_2;
%         lambda1_p_3 - lambda2_p_3;
%         lambda1_n_1 - lambda2_n_1;
%         lambda1_n_2 - lambda2_n_2;
%         lambda1_n_3 - lambda2_n_3;
    end
end

if flag1 == 1 && flag2 == 1
    figure(1)
    [C, B, D, F, Q, V, E, G, K, A, M, N] = convert(0, results1(2,1), results1(2,2), results1(2,3), params_c);
    DrawPoints(C, B, D, F, Q, V, E, G, K, A, M, N, params);
    axis equal
    axis([-1 10  -9 6]);
    xlabel('Y (m)')
    ylabel('Z (m)')
    
    figure(2)
    [C, B, D, F, Q, V, E, G, K, A, M, N] = convert(0, results2(2,1), results2(2,2), results2(2,3), params_c);
    DrawPoints(C, B, D, F, Q, V, E, G, K, A, M, N, params);
    axis equal
    axis([-1 10  -9 6]);
    xlabel('Y (m)')
    ylabel('Z (m)')
end
