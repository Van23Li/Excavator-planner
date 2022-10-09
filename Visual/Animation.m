function Animation(nodes,cfg,rate)

videoFWriter1 = VideoWriter('.\test1.mp4','MPEG-4');
videoFWriter1.FrameRate=rate;
open(videoFWriter1);

videoFWriter2 = VideoWriter('.\test2.mp4','MPEG-4');
videoFWriter2.FrameRate=rate;
open(videoFWriter2);

params = parameters();
params.phi_0 = -2.5;
params_c = parameters_cal(params);
syms a2 a3 a4
MAP = [];
dhparams_sym = [0	0	0	0;
    a2	0	0	0;
    a3	0	0	0;
    a4	0	0	0;];
dhparams = double(subs(dhparams_sym,[a2,a3,a4],[params_c.l_CF/1000,params_c.l_FQ/1000,params_c.l_QV/1000]));
step = 0.3;
step_phi = 0.1 * pi;


%%
for i=length(nodes):-1:1
    x = nodes(i).x;
    y = nodes(i).y;
    phi = nodes(i).theta+pi;
    [results1, flag1, result_num1] = ik(dhparams, x, y, phi+params.zeta,params);
    
    if flag1 == 1
        [flag1, flag1_p, flag1_n] = exam_reach(results1, params_c);
        
    end
    
    if flag1 == 1
        [C, B, D, F, Q, V, E, G, K, A, M, N] = convert(0, results1(2,1), results1(2,2), results1(2,3), params_c);
        DrawPoints(C, B, D, F, Q, V, E, G, K, A, M, N, params,1);
        rectangle('Position',[cfg.ObstPoint(1,1),cfg.ObstPoint(2,2),cfg.ObstPoint(2,1)-cfg.ObstPoint(1,1),cfg.ObstPoint(1,2)-cfg.ObstPoint(2,2)],'EdgeColor','r');
        axis equal
        axis([-1 10  -9 6]);
        xlabel('Y (m)')
        ylabel('Z (m)')
        hold off
        img = getframe(gcf);
        writeVideo(videoFWriter1,img);
    end
    
    
end
close(videoFWriter1);


end