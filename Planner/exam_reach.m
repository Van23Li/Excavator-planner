function [flag, flag_p, flag_n] = exam_reach(results, params_c)
%%
flag = 1;
flag_p = 1;
flag_n = 1;
if size(results,1) == 1
    if results(1,1) < params_c.phi_1_min || results(1,1) > params_c.phi_1_max ||...
            results(1,2) < params_c.phi_2_min || results(1,2) > params_c.phi_2_max ||...
            results(1,3) < params_c.phi_3_min || results(1,3) > params_c.phi_3_max
        flag = 0; flag_p = 0; flag_n = 0;
    end
    
elseif size(results,1) == 2
    if(results(1,1) < params_c.phi_1_min || results(1,1) > params_c.phi_1_max ||...
            results(1,2) < params_c.phi_2_min || results(1,2) > params_c.phi_2_max ||...
            results(1,3) < params_c.phi_3_min || results(1,3) > params_c.phi_3_max)
        flag_p = 0;
    end
    if(results(2,1) < params_c.phi_1_min || results(2,1) > params_c.phi_1_max ||...
            results(2,2) < params_c.phi_2_min || results(2,2) > params_c.phi_2_max ||...
            results(2,3) < params_c.phi_3_min || results(2,3) > params_c.phi_3_max)
        flag_n = 0;
    end
    if flag_p==0 && flag_n==0
        flag = 0;
    end
end
end