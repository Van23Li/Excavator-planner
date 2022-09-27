clear all
close all
%%
Configure.params = parameters();
Configure.params.phi_0 = 0;
Configure.params_c = parameters_cal(Configure.params);
syms a2 a3 a4
dhparams_sym = [0	0	0	0;
    a2	0	0	0;
    a3	0	0	0;
    a4	0	0	0;];

Configure.dhparams = double(subs(dhparams_sym,[a2,a3,a4],[Configure.params_c.l_CF/1000,Configure.params_c.l_FQ/1000,Configure.params_c.l_QV/1000]));
%%
theta1 = 0;
theta2 = 0;
theta3 = 0;
origin = [1,0,0,Configure.dhparams(4,1);
          0,1,0,0;
          0,0,1,0;
          0,0,0,1];
      
path = [];
angle = [];
for i = 1 : length(theta1)
    result = fk2(Configure.dhparams, theta1(i), theta2(i), theta3(i),origin,Configure.params);
    angle = [angle; tr2rpy(result(1:3,1:3))];
    path = [path; result(1,4), result(2,4)];
end
angle = angle - Configure.params.zeta;
% figure(1);plot(path(:,1),path(:,2),'*r'); 
figure(1);quiver(path(:,1),path(:,2),cos(angle(:,3)),sin(angle(:,3)));
tr2rpy(result(1:3,1:3))