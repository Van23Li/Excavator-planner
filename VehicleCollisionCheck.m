function isCollision = VehicleCollisionCheck(pVec,obstpoint,Vehicle)
params = parameters();
params.phi_0 = 0;
params_c = parameters_cal(params);
syms a2 a3 a4
dhparams_sym = [0	0	0	0;
    a2	0	0	0;
    a3	0	0	0;
    a4	0	0	0;];
dhparams = double(subs(dhparams_sym,[a2,a3,a4],[params_c.l_CF/1000,params_c.l_FQ/1000,params_c.l_QV/1000]));

[results1, flag1, result_num1] = ik(dhparams, pVec(1), pVec(2), pVec(3)+pi+params.zeta,params);
if flag1
    [C, B, D, F, Q, V, E, G, K, A, M, N] = convert_bucket(0, results1(2,1), results1(2,2), results1(2,3), params_c);
    Z = DrawPoints(C, B, D, F, Q, V, E, G, K, A, M, N, params, 0);
else
    error('wrong pose');
end


obs1 = [obstpoint(1,1),obstpoint(1,2);
    obstpoint(1,1),obstpoint(2,2);
    obstpoint(2,1),obstpoint(2,2)];
obs2 = [obstpoint(1,1),obstpoint(1,2);
    obstpoint(2,1),obstpoint(1,2);
    obstpoint(2,1),obstpoint(2,2)];
% 记录构成挖掘机模型的直线的起止坐标
Rect = [];
Rect(end+1,:) = V';
Rect(end+1,:) = Q';
Rect(end+1,:) = Z';
if triangle_intersection2(Rect/1000,obs1)||triangle_intersection(Rect/1000,obs2)
    isCollision = true;
    return;
else
    Rect = [];
    Rect(end+1,:) = K';
    Rect(end+1,:) = Q';
    Rect(end+1,:) = Z';
    if triangle_intersection2(Rect/1000,obs1)||triangle_intersection(Rect/1000,obs2)
        isCollision = true;
        return;
        %%
    else
        Rect = [];
        Rect(end+1,:) = F';
        Rect(end+1,:) = Q';
        Rect(end+1,:) = G';
        if triangle_intersection2(Rect/1000,obs1)||triangle_intersection(Rect/1000,obs2)
            isCollision = true;
            return;
        else
            Rect = [];
            Rect(end+1,:) = F';
            Rect(end+1,:) = D';
            Rect(end+1,:) = E';
            if triangle_intersection2(Rect/1000,obs1)||triangle_intersection(Rect/1000,obs2)
                isCollision = true;
                return;
            else
                Rect = [];
                Rect(end+1,:) = A';
                Rect(end+1,:) = C';
                Rect(end+1,:) = B';
                if triangle_intersection2(Rect/1000,obs1)||triangle_intersection(Rect/1000,obs2)
                    isCollision = true;
                    return;
                else
                    Rect = [];
                    Rect(end+1,:) = E';
                    Rect(end+1,:) = F';
                    Rect(end+1,:) = G';
                    if triangle_intersection2(Rect/1000,obs1)||triangle_intersection(Rect/1000,obs2)
                        isCollision = true;
                        return;
                    else
                        Rect = [];
                        Rect(end+1,:) = G';
                        Rect(end+1,:) = M';
                        Rect(end+1,:) = N';
                        if triangle_intersection2(Rect/1000,obs1)||triangle_intersection(Rect/1000,obs2)
                            isCollision = true;
                            return;
                        else
                            Rect = [];
                            Rect(end+1,:) = M';
                            Rect(end+1,:) = N';
                            Rect(end+1,:) = K';
                            if triangle_intersection2(Rect/1000,obs1)||triangle_intersection(Rect/1000,obs2)
                                isCollision = true;
                                return;
                            else
                                Rect = [];
                                Rect(end+1,:) = N';
                                Rect(end+1,:) = Q';
                                Rect(end+1,:) = K';
                                if triangle_intersection2(Rect/1000,obs1)||triangle_intersection(Rect/1000,obs2)
                                    isCollision = true;
                                    return;
                                end
                            end
                        end
                    end
                end
            end
        end
        %%
    end
    
end
%%
% plot([Q(1)/1000,V(1)/1000],[Q(2)/1000,V(2)/1000],'r','LineWidth',1);
% plot([Z(1)/1000,V(1)/1000],[Z(2)/1000,V(2)/1000],'r','LineWidth',1);
% plot([Q(1)/1000,Z(1)/1000],[Q(2)/1000,Z(2)/1000],'r','LineWidth',1);
% plot([Q(1)/1000,K(1)/1000],[Q(2)/1000,K(2)/1000],'r','LineWidth',1);
% plot([Q(1)/1000,Z(1)/1000],[Q(2)/1000,Z(2)/1000],'r','LineWidth',1);
% plot([K(1)/1000,Z(1)/1000],[K(2)/1000,Z(2)/1000],'r','LineWidth',1);

% DrawPoints_collision(C, B, D, F, Q, V, E, G, K, A, M, N, params);
%%
isCollision = false;

end

