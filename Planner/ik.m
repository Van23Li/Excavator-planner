function [results, flag, result_num] = ik(dhparams, x, y, phi,params)
%%
x = x-dhparams(4,1)*cos(phi);
y = y-dhparams(4,1)*sin(phi);
phi = phi;

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
    
    results = wrapToPi(results);
end

end
