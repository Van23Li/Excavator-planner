function [results, flag, result_num] = ik(dhparams, x, y, phi,params)
%输入：[x, y, phi]-Tool坐标系的坐标和角度
%给定Tool位置及方向，计算逆解
%%
%转化为腕部坐标系坐标和角度
x = x-dhparams(4,1)*cos(phi);
y = y-dhparams(4,1)*sin(phi);
phi = phi;

%设定参数
flag = 1;
result_num = 0;
l1 = dhparams(2,1);
l2 = dhparams(3,1);

c2 = (x^2 + y^2 - l1^2 - l2^2)/(2 * l1 * l2);
if abs(c2)>1
    flag = 0;
    results = [];
else
    s2 = sqrt(1 - c2^2);
    theta2 = atan2(s2, c2);
    theta2_n = -atan2(s2, c2);
    
    if x == 0 && y == 0
        theta1 = inf;
        theta3 = inf;
    else
        k1 = l1 + l2 * c2;
        k2 = l2 * s2;
        theta1 = atan2(y,x) - atan2(k2,k1);
        theta3 = phi - theta1 - theta2;
    end
    
    if c2^2 == 1
        result_num = 1;
        results = [theta1, theta2, theta3];
    else
        result_num = 2;
        theta1_n = atan2(y,x) - atan2(-k2,k1);
        theta2_n = atan2(-s2, c2);
        theta3_n = phi - theta1_n - theta2_n;
        
        results(1,:) = [theta1, theta2, theta3];
        results(2,:) = [theta1_n, theta2_n, theta3_n];
    end
    
    %放缩至-pi到pi
    results = wrapToPi(results);
    %     for i = 1:size(results,1)
    %         for j = 1:size(results,2)
    %             while results(i,j)>2*pi
    %                 results(i,j) = results(i,j)-2*pi;
    %             end
    %             while results(i,j)<0
    %                 results(i,j) = results(i,j)+2*pi;
    %             end
    %             if results(i,j)>pi
    %                 results(i,j) = results(i,j) - 2*pi;
    %             end
    %         end
    %     end
end

end
