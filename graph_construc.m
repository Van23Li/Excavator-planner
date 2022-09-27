%计算并保存挖掘机的reachable map
%%
close all;
clear all;
cal_num = 2; %逆解计算一次还是两次
params = parameters();
params.phi_0 = 0; % 转台绕z轴转动的角度

params_c = parameters_cal(params);

%%
syms a2 a3 a4
MAP = [];

%D-H参数
dhparams_sym = [0	0	0	0;
    a2	0	0	0;
    a3	0	0	0;
    a4	0	0	0;];
dhparams = double(subs(dhparams_sym,[a2,a3,a4],[params.CF/1000,params.FQ/1000,params.QV/1000]));
%%
%%%%%%%%%%%%%%%%%%%%%%%%%循环，构建graph
step = 0.3;     %x,y步长
step_phi = 0.1 * pi;        %角度步长
i_int = 0;


%画矩形
for i = -0.5 : step : 10-step
    rectangle('Position',[i -9 step 15]);
end
for j = -9 : step : 6-step
    rectangle('Position',[-0.5 j 10.5 step]);
end
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
        
        if cal_num == 2
            %%
            for phi = -pi : step_phi : pi-step_phi
                phi_int = phi_int + 1;
                
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
                        lambda1 = (abs(lambda1_p_1 - lambda2_p_1) + abs(lambda1_n_1 - lambda2_n_1))/2;
                        lambda2 = (abs(lambda1_p_2 - lambda2_p_2) + abs(lambda1_n_2 - lambda2_n_2))/2;
                        lambda3 = (abs(lambda1_p_3 - lambda2_p_3) + abs(lambda1_n_3 - lambda2_n_3))/2;
                        
                        MAP(i_int,j_int,phi_int) = 1;
                        MAP_lambda1(i_int,j_int,phi_int) = lambda1;
                        MAP_lambda2(i_int,j_int,phi_int) = lambda2;
                        MAP_lambda3(i_int,j_int,phi_int) = lambda3;
                        
                        len_lambda = ((45 - lambda1-0)/(45-0) + (34 - lambda2-0)/(34-0) + (41 - lambda3-0)/(41-0))/3;
%                         len_lambda = step/2;
                        
                        if phi_int - phi_int_pre == 1
                            if k == 0
                                k=k+1;
                                k_len = k_len + 1;
                                sector_angle(k,1) = phi;
                                sector_length(k,1) = len_lambda;
                            end
                            sector_angle(k,2) = phi;
                            k_len = k_len + 1;
                            sector_length(k,k_len) = len_lambda;
                            phi_int_pre = phi_int;
                        else
                            k_len = 0;
                            k = k + 1;
                            k_len = k_len + 1;
                            sector_angle(k,1) = phi;
                            sector_angle(k,2) = phi;
                            sector_length(k,k_len) = len_lambda;
                            %                         sector_length(k,k_len+1) = len_lambda;
                            phi_int_pre = phi_int;
                        end
                    else
                        MAP(i_int,j_int,phi_int) = 0;
                        MAP_lambda1(i_int,j_int,phi_int) = 0;
                        MAP_lambda2(i_int,j_int,phi_int) = 0;
                        MAP_lambda3(i_int,j_int,phi_int) = 0;
                    end
                end
            end
            %%%%%%%%%%%%%%%%一起画%%%%%%%%%%%%%%%%%%%%%%%%%
            if ~isempty(sector_angle)
                for num = 1 : size(sector_angle,1)
                    %                 sector(x,y,sector_angle(num,1)+pi,sector_angle(num,2)+pi,step/2);
                    sector_len(x,y,sector_angle(num,1)+pi,sector_angle(num,2)+pi,sector_length(num,:),step_phi);
                    hold on
                    axis([-0.5 10  -9 6]);
                    axis equal
                end
            end
            %%
        elseif cal_num == 1
            
            for phi = -pi : step_phi : pi-step_phi
                phi_int = phi_int + 1;
                
                %对两个对应的位姿进行逆解
                [results, flag, result_num] = ik(dhparams, x, y, phi+params.zeta,params);
                
                %判断是否超过挖掘机限制
                if flag == 1
                    [flag, flag_p, flag_n] = exam_reach(results, params_c);
                    
                    if flag == 1
                        lambda_p_1 = 0; lambda_p_2 = 0; lambda_p_3 = 0;
                        lambda_n_1 = 0; lambda_n_2 = 0; lambda_n_3 = 0;
                        if flag_p == 1
                            [lambda_p_1, lambda_p_2, lambda_p_3] = work2drive(results(1,1), results(1,2), results(1,3), params_c);
                        end
                        if flag_n == 1
                            [lambda_n_1, lambda_n_2, lambda_n_3] = work2drive(results(2,1), results(2,2), results(2,3), params_c);
                        end
                        %                         lambda1 = (abs(lambda1_p_1 - lambda2_p_1) + abs(lambda1_n_1 - lambda2_n_1))/2;
                        %                         lambda2 = (abs(lambda1_p_2 - lambda2_p_2) + abs(lambda1_n_2 - lambda2_n_2))/2;
                        %                         lambda3 = (abs(lambda1_p_3 - lambda2_p_3) + abs(lambda1_n_3 - lambda2_n_3))/2;
                        
                        MAP(i_int,j_int,phi_int) = 1;
                        %                         MAP_lambda1(i_int,j_int,phi_int) = lambda1;
                        %                         MAP_lambda2(i_int,j_int,phi_int) = lambda2;
                        %                         MAP_lambda3(i_int,j_int,phi_int) = lambda3;
                        
                        %                     len_lambda = ((45 - lambda1-0)/(45-0) + (34 - lambda2-0)/(34-0) + (41 - lambda3-0)/(41-0))/3;
                        len_lambda = step/2;
                        
                        if phi_int - phi_int_pre == 1
                            if k == 0
                                k=k+1;
                                k_len = k_len + 1;
                                sector_angle(k,1) = phi;
                                sector_length(k,1) = len_lambda;
                            end
                            sector_angle(k,2) = phi;
                            k_len = k_len + 1;
                            sector_length(k,k_len) = len_lambda;
                            phi_int_pre = phi_int;
                        else
                            k_len = 0;
                            k = k + 1;
                            k_len = k_len + 1;
                            sector_angle(k,1) = phi;
                            sector_angle(k,2) = phi;
                            sector_length(k,k_len) = len_lambda;
                            %                         sector_length(k,k_len+1) = len_lambda;
                            phi_int_pre = phi_int;
                        end
                    end
                else
                    MAP(i_int,j_int,phi_int) = 0;
                    %                         MAP_lambda1(i_int,j_int,phi_int) = 0;
                    %                         MAP_lambda2(i_int,j_int,phi_int) = 0;
                    %                         MAP_lambda3(i_int,j_int,phi_int) = 0;
                end
            end
            %%%%%%%%%%%%%%%%一起画%%%%%%%%%%%%%%%%%%%%%%%%%
            if ~isempty(sector_angle)
                for num = 1 : size(sector_angle,1)
                    %                 sector(x,y,sector_angle(num,1)+pi,sector_angle(num,2)+pi,step/2);
                    sector_len(x,y,sector_angle(num,1)+pi,sector_angle(num,2)+pi,sector_length(num,:),step_phi);
                    hold on
                    axis([-0.5 10  -9 6]);
                    axis equal
                end
            end
        end
        %%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
end
%%
if cal_num == 2
    save MAP_01_multi MAP
    save MAP_lambda1 MAP_lambda1
    save MAP_lambda2 MAP_lambda2
    save MAP_lambda3 MAP_lambda3
elseif cal_num == 1
    save MAP_01 MAP
end
