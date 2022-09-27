%加载保存的数据直接画图
%%
clear all
close all
load MAP_03_multi.mat
load MAP_lambda1.mat
load MAP_lambda2.mat
load MAP_lambda3.mat
load len_lambda_list.mat
MAP = cat(3,MAP,MAP(:,:,1));
MAP_lambda1 = cat(3,MAP_lambda1,MAP_lambda1(:,:,1));
MAP_lambda2 = cat(3,MAP_lambda2,MAP_lambda2(:,:,1));
MAP_lambda3 = cat(3,MAP_lambda3,MAP_lambda3(:,:,1));
step = 0.3;     %x,y步长
step_phi = 0.1 * pi;        %角度步长
i_int = 0;
%         len_lambda_list = [];
for i = -0.5 : step : 10-step
    i_int = i_int + 1;
    x = i + step/2;
    j_int = 0;
    for j = -9 : step : 6-step
        j_int = j_int + 1;
        y = j + step/2;
        
        %画矩形
        rectangle('Position',[i j step step],'EdgeColor','y');
        axis([-0.5 10  -9 6]);
        hold on
        
        phi_int = 0;
        phi_int_pre = Inf;
        k = 0;
        k_len = 0;
        sector_angle = [];
        sector_length = [];
        
        for phi = -pi : step_phi : pi
            phi_int = phi_int + 1;
            if MAP(i_int,j_int,phi_int) == 1
                %ToDo: 变长度
                lambda1 = MAP_lambda1(i_int,j_int,phi_int);
                lambda2 = MAP_lambda2(i_int,j_int,phi_int);
                lambda3 = MAP_lambda3(i_int,j_int,phi_int);
                len_lambda = ((lambda1-min(MAP_lambda1(:)))/(max(MAP_lambda1(:))-min(MAP_lambda1(:))) + (lambda2-min(MAP_lambda2(:)))/(max(MAP_lambda2(:))-min(MAP_lambda2(:))) + (lambda3-min(MAP_lambda3(:)))/(max(MAP_lambda3(:))-min(MAP_lambda3(:))))/3;
%                 len_lambda_list = [len_lambda_list, len_lambda];
%                 len_lambda = (len_lambda - min(len_lambda_list(:)))/(max(len_lambda_list(:)) - min(len_lambda_list(:))) * step/2;
                len_lambda = step/2*1/2 + (len_lambda - min(len_lambda_list(:)))/(max(len_lambda_list(:)) - min(len_lambda_list(:))) * step/2*1/2;
                
                
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
save len_lambda_list len_lambda_list