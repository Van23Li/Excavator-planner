clear all
close all
for j = 1:100
    close all
    %%
    size = [100 200 600 300];
    Configure.params = parameters();
    Configure.params.phi_0 = 0;
    Configure.params_c = parameters_cal(Configure.params);
    syms a2 a3 a4
    dhparams_sym = [0	0	0	0;
        a2	0	0	0;
        a3	0	0	0;
        a4	0	0	0;];
    
    Configure.dhparams = double(subs(dhparams_sym,[a2,a3,a4],[Configure.params_c.l_CF/1000,Configure.params_c.l_FQ/1000,Configure.params_c.l_QV/1000]));
    %% 人工设置theta
%     theta1 = (Configure.params_c.phi_1_max - Configure.params_c.phi_1_min)/2 * ...
%                 sin(pi/25*(1:50)) + (Configure.params_c.phi_1_max - Configure.params_c.phi_1_min)/2;
%     theta2 = (Configure.params_c.phi_2_max - Configure.params_c.phi_2_min)/2 * ...
%                 sin(pi/25*(1:50)) + (Configure.params_c.phi_2_max - Configure.params_c.phi_2_min)/2;
%     theta3 = (Configure.params_c.phi_3_max - Configure.params_c.phi_3_min)/2 * ...
%                 sin(pi/25*(1:50)) + (Configure.params_c.phi_3_max - Configure.params_c.phi_3_min)/2;
%             
%     figure(10);plot(theta1,'*');
%     set(gcf, 'Position', size);
%     figure(20);plot(theta2,'-');
%     set(gcf, 'Position', size);
%     figure(30);plot(theta3,'-');
%     set(gcf, 'Position', size);
    
    %% 插值得到theta
%         theta1 = linspace(Configure.params_c.phi_1_min, Configure.params_c.phi_1_max, 50);
%         theta2 = linspace(Configure.params_c.phi_2_min, Configure.params_c.phi_2_max, 50);
%         theta3 = linspace(Configure.params_c.phi_3_min, Configure.params_c.phi_3_max, 50);
    
        % theta1 = ones(50) * 0;
        % theta2 = ones(50) * 0;
        % theta3 = ones(50) * 0;
        
        beilv = 4;
        
        theta1_x = [0, 5 + rand * (10 - 5), 10 + rand * (20 - 10), 20 + rand * (30 - 20), ...
                        30 + rand * (40 - 30), 40 + rand * (45 - 40), 50];
        theta2_x = [0, 5 + rand * (10 - 5), 10 + rand * (20 - 10), 20 + rand * (30 - 20), ...
                        30 + rand * (40 - 30), 40 + rand * (45 - 40), 50];
%         theta2_x = [0	7.56573917	10.0481381	23.56711919	39.73021728	40.72328814	50];

        theta3_x = [0, 5 + rand * (10 - 5), 10 + rand * (20 - 10), 20 + rand * (30 - 20), ...
                        30 + rand * (40 - 30), 40 + rand * (45 - 40), 50];
        theta1_y = (Configure.params_c.phi_1_max + Configure.params_c.phi_1_min)/2 + (2*rand(1,7)-1) * ...
                        (Configure.params_c.phi_1_max - Configure.params_c.phi_1_min) / beilv;
        theta2_y = (Configure.params_c.phi_2_max + Configure.params_c.phi_2_min)/2 + (2*rand(1,7)-1) * ...
                        (Configure.params_c.phi_2_max - Configure.params_c.phi_2_min) / beilv;
        theta3_y = (Configure.params_c.phi_3_max + Configure.params_c.phi_3_min)/2 + (2*rand(1,7)-1) * ...
                        (Configure.params_c.phi_3_max - Configure.params_c.phi_3_min) / beilv;
        theta1_xx = linspace(0,50,100);
        theta2_xx = linspace(0,50,100);
        theta3_xx = linspace(0,50,100);
        theta1_yy = spline(theta1_x,[0 theta1_y 0]);
        theta2_yy = spline(theta2_x,[0 theta2_y 0]);
        theta3_yy = spline(theta3_x,[0 theta3_y 0]);
        
        figure(10);plot(theta1_x,theta1_y,'o',theta1_xx,ppval(theta1_yy,theta1_xx),'-','LineWidth',1.5);
%         hold on; plot([0,50], [Configure.params_c.phi_1_max, Configure.params_c.phi_1_max]);
%         hold on; plot([0,50], [Configure.params_c.phi_1_min, Configure.params_c.phi_1_min]);
        axis([0 50 min(ppval(theta1_yy,theta1_xx))-0.2*abs(min(ppval(theta1_yy,theta1_xx))) max(ppval(theta1_yy,theta1_xx)) + 0.2 * abs(max(ppval(theta1_yy,theta1_xx)))]);
        set(gcf, 'Position', size);
        set(gca, 'FontSize', 25);
        
        figure(20);plot(theta2_x,theta2_y,'o',theta2_xx,ppval(theta2_yy,theta2_xx),'-','LineWidth',1.5);
%         hold on; plot([0,50], [Configure.params_c.phi_2_max, Configure.params_c.phi_2_max]);
%         hold on; plot([0,50], [Configure.params_c.phi_2_min, Configure.params_c.phi_2_min]);
        axis([0 50 min(ppval(theta2_yy,theta2_xx))-0.2*abs(min(ppval(theta2_yy,theta2_xx))) max(ppval(theta2_yy,theta2_xx)) + 0.2 * abs(max(ppval(theta2_yy,theta2_xx)))]);
        set(gcf, 'Position', size);
        set(gca, 'FontSize', 25);
        
        figure(30);plot(theta3_x,theta3_y,'o',theta3_xx,ppval(theta3_yy,theta3_xx),'-','LineWidth',1.5);
%         hold on; plot([0,50], [Configure.params_c.phi_3_max, Configure.params_c.phi_3_max]);
%         hold on; plot([0,50], [Configure.params_c.phi_3_min, Configure.params_c.phi_3_min]);
        axis([0 50 min(ppval(theta3_yy,theta3_xx))-0.2*abs(min(ppval(theta3_yy,theta3_xx))) max(ppval(theta3_yy,theta3_xx)) + 0.2 * abs(max(ppval(theta3_yy,theta3_xx)))]);
        set(gcf, 'Position', size);
        set(gca, 'FontSize', 25);
        
        theta1 = ppval(theta1_yy,theta1_xx);
        theta2 = ppval(theta2_yy,theta2_xx);
        theta3 = ppval(theta3_yy,theta3_xx);
    %%
    origin = [1,0,0,Configure.dhparams(4,1);
        0,1,0,0;
        0,0,1,0;
        0,0,0,1];
    %% 基于顺运动学，画task-space图
    path = [];
    angle = [];
    for i = 1 : length(theta1)
        result = fk2(Configure.dhparams, theta1(i), theta2(i), theta3(i),origin,Configure.params);
        angle = [angle; tr2rpy(result(1:3,1:3))];
        path = [path; result(1,4), result(2,4)];
    end
    % figure(1);plot(path(:,1),path(:,2));
    % axis equal
    % axis([-1 10  -9 8]);
    % xlabel('Y (m)')
    % ylabel('Z (m)')
    angle = angle - Configure.params.zeta;
    % figure(2);quiver(path(:,1),path(:,2),cos(angle(:,3)),sin(angle(:,3)));
    % axis equal
    % axis([-1 10  -9 8]);
    % xlabel('Y (m)')
    % ylabel('Z (m)')
    
    %% 画挖掘机
    if 0
        for i = 1 : length(theta1)
            figure(11)
            [C, B, D, F, Q, V, E, G, K, A, M, N] = convert(0, theta1(i), theta2(i), theta3(i), Configure.params_c);
            DrawPoints(C, B, D, F, Q, V, E, G, K, A, M, N, Configure.params,1);
            axis equal
            axis([-1 10  -9 8]);
            xlabel('Y (m)')
            ylabel('Z (m)')
            %         img = getframe(gcf);
            %         writeVideo(videoFWriter1,img);
            hold off
        end
    end
    
    %% 基于几何关系，求解液压缸位移，绘图
    lambda1_list = [];
    lambda2_list = [];
    lambda3_list = [];
    for i = 1 : length(theta1)
        [lambda1, lambda2, lambda3] = work2drive(theta1(i), theta2(i), theta3(i), Configure.params_c);
        lambda1_list = [lambda1_list;lambda1 / 1000];
        lambda2_list = [lambda2_list;lambda2 / 1000];
        lambda3_list = [lambda3_list;lambda3 / 1000];
    end
    figure(3);plot(linspace(0,50,100),lambda1_list,'LineWidth',1.5);
    axis([0 50 min(lambda1_list)-0.2*abs(max(lambda1_list) - min(lambda1_list)) max(lambda1_list) + 0.2 * abs(max(lambda1_list) - min(lambda1_list))]);
    set(gcf, 'Position', size);
    set(gca, 'FontSize', 25);
        
    figure(4);plot(linspace(0,50,100),lambda2_list,'LineWidth',1.5);
    axis([0 50 min(lambda2_list)-0.2*abs(max(lambda2_list) - min(lambda2_list)) max(lambda2_list) + 0.2 * abs(max(lambda2_list) - min(lambda2_list))]);
    set(gcf, 'Position', size);
    set(gca, 'FontSize', 25);
    
    figure(5);plot(linspace(0,50,100),lambda3_list,'LineWidth',1.5);
    axis([0 50 min(lambda3_list)-0.2*abs(max(lambda3_list) - min(lambda3_list)) max(lambda3_list) + 0.2 * abs(max(lambda3_list) - min(lambda3_list))]);
    set(gcf, 'Position', size);
    set(gca, 'FontSize', 25);
    
    %% 量化分析
    % % 关节角度
    % dtheta1_dx = diff(theta1);
    % ddtheta1_dx = diff(dtheta1_dx);
    % if isempty(ddtheta1_dx)
    %     Smooth(1,1) = 0;
    % else
    %     Smooth(1,1) = sum(ddtheta1_dx.^2)/length(ddtheta1_dx);
    % end
    %
    % dtheta2_dx = diff(theta2);
    % ddtheta2_dx = diff(dtheta2_dx);
    % if isempty(ddtheta2_dx)
    %     Smooth(1,2) = 0;
    % else
    %     Smooth(1,2) = sum(ddtheta2_dx.^2)/length(ddtheta2_dx);
    % end
    %
    % dtheta3_dx = diff(theta3);
    % ddtheta3_dx = diff(dtheta3_dx);
    % if isempty(ddtheta3_dx)
    %     Smooth(1,3) = 0;
    % else
    %     Smooth(1,3) = sum(ddtheta3_dx.^2)/length(ddtheta3_dx);
    % end
    %
    % % 液压缸位移
    % dlambda1_dx = diff(lambda1_list);
    % ddlambda1_dx = diff(dlambda1_dx);
    % if isempty(ddlambda1_dx)
    %     Smooth(1,4) = 0;
    % else
    %     Smooth(1,4) = sum(ddlambda1_dx.^2)/length(ddlambda1_dx);
    % end
    %
    % dlambda2_dx = diff(lambda2_list);
    % ddlambda2_dx = diff(dlambda2_dx);
    % if isempty(ddlambda2_dx)
    %     Smooth(1,5) = 0;
    % else
    %     Smooth(1,5) = sum(ddlambda2_dx.^2)/length(ddlambda2_dx);
    % end
    %
    % dlambda3_dx = diff(lambda3_list);
    % ddlambda3_dx = diff(dlambda3_dx);
    % if isempty(ddlambda3_dx)
    %     Smooth(1,6) = 0;
    % else
    %     Smooth(1,6) = sum(ddlambda3_dx.^2)/length(ddlambda3_dx);
    % end
    %
    % Smooth
    %%
    %% 量化分析
    % 关节角度
    dtheta1_b = diff(theta1(2:end));
    dtheta1_f = diff(theta1(1:end-1));
    if isempty(dtheta1_b) & isempty(dtheta1_f)
        Smooth(1,1) = 0;
    else
%                 Smooth(1,1) = sum((dtheta1_b - dtheta1_f).^2)/length(dtheta1_b);
        Smooth(1,1) = length(dtheta1_b) / trapz(1:length(dtheta1_b),(dtheta1_b - dtheta1_f).^2);
    end
    
    dtheta2_b = diff(theta2(2:end));
    dtheta2_f = diff(theta2(1:end-1));
    if isempty(dtheta2_b) & isempty(dtheta2_f)
        Smooth(1,2) = 0;
    else
%                 Smooth(1,2) = sum((dtheta2_b - dtheta2_f).^2)/length(dtheta2_b);
        Smooth(1,2) = length(dtheta2_b) / trapz(1:length(dtheta2_b),(dtheta2_b - dtheta2_f).^2);
    end
    
    dtheta3_b = diff(theta3(2:end));
    dtheta3_f = diff(theta3(1:end-1));
    if isempty(dtheta3_b) & isempty(dtheta3_f)
        Smooth(1,3) = 0;
    else
%                 Smooth(1,3) = sum((dtheta3_b - dtheta3_f).^2)/length(dtheta3_b);
        Smooth(1,3) = length(dtheta3_b) / trapz(1:length(dtheta3_b),(dtheta3_b - dtheta3_f).^2);
    end
    
    % 液压缸
    dlambda1_b = diff(lambda1_list(2:end));
    dlambda1_f = diff(lambda1_list(1:end-1));
    if isempty(dlambda1_b) & isempty(dlambda1_f)
        Smooth(1,4) = 0;
    else
%                 Smooth(1,4) = sum((dlambda1_b - dlambda1_f).^2)/length(dlambda1_b);
        Smooth(1,4) = length(dlambda1_b) / trapz(1:length(dlambda1_b),(dlambda1_b - dlambda1_f).^2);
    end
    
    dlambda2_b = diff(lambda2_list(2:end));
    dlambda2_f = diff(lambda2_list(1:end-1));
    if isempty(dlambda2_b) & isempty(dlambda2_f)
        Smooth(1,5) = 0;
    else
%                 Smooth(1,5) = sum((dlambda2_b - dlambda2_f).^2)/length(dlambda2_b);
        Smooth(1,5) = length(dlambda2_b) / trapz(1:length(dlambda2_b),(dlambda2_b - dlambda2_f).^2);
    end
    
    dlambda3_b = diff(lambda3_list(2:end));
    dlambda3_f = diff(lambda3_list(1:end-1));
    if isempty(dlambda3_b) & isempty(dlambda3_f)
        Smooth(1,6) = 0;
    else
%                 Smooth(1,6) = sum((dlambda3_b - dlambda3_f).^2)/length(dlambda3_b);
        Smooth(1,6) = length(dlambda3_b) / trapz(1:length(dlambda3_b),(dlambda3_b - dlambda3_f).^2);
    end
    
%     Smooth
    if Smooth(3) >= Smooth(6)
        a=3
    end
    if Smooth(1) >= Smooth(4)
        a=3
    end
    if Smooth(2) >= Smooth(5)
        a=3
    end
%     if Smooth(1) > Smooth(4) & Smooth(2) > Smooth(5) & Smooth(3) > Smooth(6)
%         [(Smooth(1) - Smooth(4))/Smooth(4), (Smooth(2) - Smooth(5))/Smooth(5),(Smooth(3) - Smooth(6))/Smooth(6)]
% %         a=3
%     end
%     if (Smooth(4) - Smooth(1))/Smooth(1) >0.6 & (Smooth(5) - Smooth(2))/Smooth(2) > 0.8 & (Smooth(6) - Smooth(3))/Smooth(3) > 4
%         a=3
%     end
end