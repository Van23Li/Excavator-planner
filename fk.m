function results = ik(dhparams, theta1, theta2, theta3,origin,params)
T1 = [1,           0,           0, dhparams(1,1);
      0,           1,           0, 0;
      0,           0,           1, 0;
      0,           0,           0, 1];
T2 = [cos(theta1), sin(theta1), 0, dhparams(2,1);
      sin(theta1), cos(theta1), 0, 0;
      0,           0,           1, 0;
      0,           0,           0, 1];
T3 = [cos(theta2), sin(theta2), 0, dhparams(3,1);
      sin(theta2), cos(theta2), 0, 0;
      0,           0,           1, 0;
      0,           0,           0, 1];
% T4 = [cos(theta3), sin(theta3), 0, dhparams(4,1);
%       sin(theta3), cos(theta3), 0, 0;
%       0,           0,           1, 0;
%       0,           0,           0, 1];
% results = T1 * T2 * T3 * T4;
results = T1 * T2 * T3 * origin;
end