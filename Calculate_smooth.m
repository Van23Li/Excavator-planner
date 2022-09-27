clear all;
close all;
size = [100 200 600 300];
Smooth = [];
Smooth_name =[];
Smooth_cow = 1;
maindir = 'C:\Users\21782\Documents\Van\课题组\李老师课题组\12-2022-Sep.-ICRA23\代码\research_map_0112_drivecost\Test'
subfolder = dir(maindir);
subfolder = subfolder(3:end);
for num = 1:length(subfolder)
    subdir = dir([maindir, '\', subfolder(num).name, '\*.mat' ]);
    for i = 1:6:length(subdir)-1
        lambda1_0 = load([subdir(i+0).folder,'\',subdir(i+0).name]);
        lambda1_10 = load([subdir(i+1).folder,'\',subdir(i+1).name]);
        lambda2_0 = load([subdir(i+2).folder,'\',subdir(i+2).name]);
        lambda2_10 = load([subdir(i+3).folder,'\',subdir(i+3).name]);
        lambda3_0 = load([subdir(i+4).folder,'\',subdir(i+4).name]);
        lambda3_10 = load([subdir(i+5).folder,'\',subdir(i+5).name]);
        
%         a = subdir(i+0).name
%         any2str = @(x) evalc('disp(x)');
%         Smooth_name(Smooth_cow,:) = any2str(a);
%         Smooth_name(Smooth_cow,:) = num2str(a);
%         Smooth_name(Smooth_cow,:) = fullfile(a);
        
        if length(lambda1_0.lambda1_list)>=10
            dlambda_b = diff(lambda1_0.lambda1_list(2:end));
            dlambda_f = diff(lambda1_0.lambda1_list(1:end-1));
            if isempty(dlambda_b)
                Smooth(Smooth_cow,1) = 0;
            else
                Smooth(Smooth_cow,1) = length(dlambda_b) / trapz(1:length(dlambda_b),(dlambda_b - dlambda_f).^2);
            end
            
            if num == 135 & i ==7
                a=2
            end
            
            dlambda_b = diff(lambda1_10.lambda1_list(2:end));
            dlambda_f = diff(lambda1_10.lambda1_list(1:end-1));
            if isempty(dlambda_b)
                Smooth(Smooth_cow,2) = 0;
            else
                Smooth(Smooth_cow,2) = length(dlambda_b) / trapz(1:length(dlambda_b),(dlambda_b - dlambda_f).^2);
            end
            
            dlambda_b = diff(lambda2_0.lambda2_list(2:end));
            dlambda_f = diff(lambda2_0.lambda2_list(1:end-1));
            if isempty(dlambda_b)
                Smooth(Smooth_cow,3) = 0;
            else
                Smooth(Smooth_cow,3) = length(dlambda_b) / trapz(1:length(dlambda_b),(dlambda_b - dlambda_f).^2);
            end
           
            dlambda_b = diff(lambda2_10.lambda2_list(2:end));
            dlambda_f = diff(lambda2_10.lambda2_list(1:end-1));
            if isempty(dlambda_b)
                Smooth(Smooth_cow,4) = 0;
            else
                Smooth(Smooth_cow,4) = length(dlambda_b) / trapz(1:length(dlambda_b),(dlambda_b - dlambda_f).^2);
            end
            
            dlambda_b = diff(lambda3_0.lambda3_list(2:end));
            dlambda_f = diff(lambda3_0.lambda3_list(1:end-1));
            if isempty(dlambda_b)
                Smooth(Smooth_cow,5) = 0;
            else
                Smooth(Smooth_cow,5) = length(dlambda_b) / trapz(1:length(dlambda_b),(dlambda_b - dlambda_f).^2);
            end
            
            
            dlambda_b = diff(lambda3_10.lambda3_list(2:end));
            dlambda_f = diff(lambda3_10.lambda3_list(1:end-1));
            if isempty(dlambda_b)
                Smooth(Smooth_cow,6) = 0;
            else
                Smooth(Smooth_cow,6) = length(dlambda_b) / trapz(1:length(dlambda_b),(dlambda_b - dlambda_f).^2);
            end
            %%
            if Smooth(Smooth_cow,1) < 1826 & Smooth(Smooth_cow,1) > 1824
                close all
                x = 0:length(lambda1_0.lambda1_list)-1;
                x0 = 0:length(lambda1_10.lambda1_list)-1;
                y1 = lambda1_0.lambda1_list;
                y2 = lambda2_0.lambda2_list;
                y3 = lambda3_0.lambda3_list;
                y10 = lambda1_10.lambda1_list;
                y20 = lambda2_10.lambda2_list;
                y30 = lambda3_10.lambda3_list;
                xx = 0:0.01:length(lambda1_0.lambda1_list)-1
                xx0 = 0:0.01:length(lambda1_10.lambda1_list)-1
                yy1 = interp1(x,y1,xx,'cubic');
                yy2 = interp1(x,y2,xx,'cubic');
                yy3 = interp1(x,y3,xx,'cubic');
                yy10 = interp1(x0,y10,xx0,'cubic');
                yy20 = interp1(x0,y20,xx0,'cubic');
                yy30 = interp1(x0,y30,xx0,'cubic');
                
                figure(100);plot(xx,yy1,'linewidth',1.5,'MarkerEdgeColor','r');
                hold on;plot(xx0,yy10,'linewidth',1.5,'MarkerEdgeColor','b');
                xlim([0,max(x(end),x0(end))]);
                ylim([1.2,1.6]);
                set(gcf, 'Position', size);
                set(gca, 'FontSize', 25);
                
                figure(200);plot(xx,yy2,'linewidth',1.5,'MarkerEdgeColor','r');
                hold on;plot(xx0,yy20,'linewidth',1.5,'MarkerEdgeColor','b');
                xlim([0,max(x(end),x0(end))]);
                ylim([1,1.6]);
                set(gcf, 'Position', size);
                set(gca, 'FontSize', 25);
                
                figure(300);plot(xx,yy3,'linewidth',1.5,'MarkerEdgeColor','r');
                hold on;plot(xx0,yy30,'linewidth',1.5,'MarkerEdgeColor','b');
                xlim([0,max(x(end),x0(end))]);
                ylim([0.8,1.4]);
                set(gcf, 'Position', size);
                set(gca, 'FontSize', 25);
                a=3
            end
         %%   
            
            if Smooth(Smooth_cow,1) < 3186 & Smooth(Smooth_cow,1) > 3185
                close all
                x = 0:length(lambda1_0.lambda1_list)-1;
                x0 = 0:length(lambda1_10.lambda1_list)-1;
                y1 = lambda1_0.lambda1_list;
                y2 = lambda2_0.lambda2_list;
                y3 = lambda3_0.lambda3_list;
                y10 = lambda1_10.lambda1_list;
                y20 = lambda2_10.lambda2_list;
                y30 = lambda3_10.lambda3_list;
                xx = 0:0.01:length(lambda1_0.lambda1_list)-1
                xx0 = 0:0.01:length(lambda1_10.lambda1_list)-1
                yy1 = interp1(x,y1,xx,'cubic');
                yy2 = interp1(x,y2,xx,'cubic');
                yy3 = interp1(x,y3,xx,'cubic');
                yy10 = interp1(x0,y10,xx0,'cubic');
                yy20 = interp1(x0,y20,xx0,'cubic');
                yy30 = interp1(x0,y30,xx0,'cubic');
                
                figure(100);plot(xx,yy1,'linewidth',1.5,'MarkerEdgeColor','r');
                hold on;plot(xx0,yy10,'linewidth',1.5,'MarkerEdgeColor','b');
                xlim([0,max(x(end),x0(end))]);
                ylim([1.2,1.6]);
                set(gcf, 'Position', size);
                set(gca, 'FontSize', 25);
                
                figure(200);plot(xx,yy2,'linewidth',1.5,'MarkerEdgeColor','r');
                hold on;plot(xx0,yy20,'linewidth',1.5,'MarkerEdgeColor','b');
                xlim([0,max(x(end),x0(end))]);
                ylim([1.1,1.5]);
                set(gcf, 'Position', size);
                set(gca, 'FontSize', 25);
                
                figure(300);plot(xx,yy3,'linewidth',1.5,'MarkerEdgeColor','r');
                hold on;plot(xx0,yy30,'linewidth',1.5,'MarkerEdgeColor','b');
                xlim([0,max(x(end),x0(end))]);
                ylim([0.8,1.4]);
                set(gcf, 'Position', size);
                set(gca, 'FontSize', 25);
                a=3
            end
         %%   
            
            if Smooth(Smooth_cow,1) < 8604 & Smooth(Smooth_cow,1) > 8603
                close all
                x = 0:length(lambda1_0.lambda1_list)-1;
                x0 = 0:length(lambda1_10.lambda1_list)-1;
                y1 = lambda1_0.lambda1_list;
                y2 = lambda2_0.lambda2_list;
                y3 = lambda3_0.lambda3_list;
                y10 = lambda1_10.lambda1_list;
                y20 = lambda2_10.lambda2_list;
                y30 = lambda3_10.lambda3_list;
                xx = 0:0.01:length(lambda1_0.lambda1_list)-1
                xx0 = 0:0.01:length(lambda1_10.lambda1_list)-1
                yy1 = interp1(x,y1,xx,'cubic');
                yy2 = interp1(x,y2,xx,'cubic');
                yy3 = interp1(x,y3,xx,'cubic');
                yy10 = interp1(x0,y10,xx0,'cubic');
                yy20 = interp1(x0,y20,xx0,'cubic');
                yy30 = interp1(x0,y30,xx0,'cubic');
                
                figure(100);plot(xx,yy1,'linewidth',1.5,'MarkerEdgeColor','r');
                hold on;plot(xx0,yy10,'linewidth',1.5,'MarkerEdgeColor','b');
                xlim([0,max(x(end),x0(end))]);
                ylim([1.3,1.6]);
                set(gcf, 'Position', size);
                set(gca, 'FontSize', 25);
                
                figure(200);plot(xx,yy2,'linewidth',1.5,'MarkerEdgeColor','r');
                hold on;plot(xx0,yy20,'linewidth',1.5,'MarkerEdgeColor','b');
                xlim([0,max(x(end),x0(end))]);
                ylim([1.3,1.7]);
                set(gcf, 'Position', size);
                set(gca, 'FontSize', 25);
                
                figure(300);plot(xx,yy3,'linewidth',1.5,'MarkerEdgeColor','r');
                hold on;plot(xx0,yy30,'linewidth',1.5,'MarkerEdgeColor','b');
                xlim([0,max(x(end),x0(end))]);
                ylim([0.8,1.2]);
                set(gcf, 'Position', size);
                set(gca, 'FontSize', 25);
                a=3
            end
            %%
            
            if Smooth(Smooth_cow,1) < 5766 & Smooth(Smooth_cow,1) > 5765
                close all
                x = 0:length(lambda1_0.lambda1_list)-1;
                x0 = 0:length(lambda1_10.lambda1_list)-1;
                y1 = lambda1_0.lambda1_list;
                y2 = lambda2_0.lambda2_list;
                y3 = lambda3_0.lambda3_list;
                y10 = lambda1_10.lambda1_list;
                y20 = lambda2_10.lambda2_list;
                y30 = lambda3_10.lambda3_list;
                xx = 0:0.01:length(lambda1_0.lambda1_list)-1
                xx0 = 0:0.01:length(lambda1_10.lambda1_list)-1
                yy1 = interp1(x,y1,xx,'cubic');
                yy2 = interp1(x,y2,xx,'cubic');
                yy3 = interp1(x,y3,xx,'cubic');
                yy10 = interp1(x0,y10,xx0,'cubic');
                yy20 = interp1(x0,y20,xx0,'cubic');
                yy30 = interp1(x0,y30,xx0,'cubic');
                
                figure(100);plot(xx,yy1,'linewidth',1.5,'MarkerEdgeColor','r');
                hold on;plot(xx0,yy10,'linewidth',1.5,'MarkerEdgeColor','b');
                xlim([0,max(x(end),x0(end))]);
%                 ylim([1.0,1.4]);
                set(gcf, 'Position', size);
                set(gca, 'FontSize', 25);
                
                figure(200);plot(xx,yy2,'linewidth',1.5,'MarkerEdgeColor','r');
                hold on;plot(xx0,yy20,'linewidth',1.5,'MarkerEdgeColor','b');
                xlim([0,max(x(end),x0(end))]);
%                 ylim([1.1,1.4]);
                set(gcf, 'Position', size);
                set(gca, 'FontSize', 25);
                
                figure(300);plot(xx,yy3,'linewidth',1.5,'MarkerEdgeColor','r');
                hold on;plot(xx0,yy30,'linewidth',1.5,'MarkerEdgeColor','b');
                xlim([0,max(x(end),x0(end))]);
%                 ylim([1.0,1.3]);
                set(gcf, 'Position', size);
                set(gca, 'FontSize', 25);
                a=3
            end
          
            %%
            
            if Smooth(Smooth_cow,1) < 6131 & Smooth(Smooth_cow,1) > 6130
                close all
                x = 0:length(lambda1_0.lambda1_list)-1;
                x0 = 0:length(lambda1_10.lambda1_list)-1;
                y1 = lambda1_0.lambda1_list;
                y2 = lambda2_0.lambda2_list;
                y3 = lambda3_0.lambda3_list;
                y10 = lambda1_10.lambda1_list;
                y20 = lambda2_10.lambda2_list;
                y30 = lambda3_10.lambda3_list;
                xx = 0:0.01:length(lambda1_0.lambda1_list)-1
                xx0 = 0:0.01:length(lambda1_10.lambda1_list)-1
                yy1 = interp1(x,y1,xx,'cubic');
                yy2 = interp1(x,y2,xx,'cubic');
                yy3 = interp1(x,y3,xx,'cubic');
                yy10 = interp1(x0,y10,xx0,'cubic');
                yy20 = interp1(x0,y20,xx0,'cubic');
                yy30 = interp1(x0,y30,xx0,'cubic');
                
                figure(100);plot(xx,yy1,'linewidth',1.5,'MarkerEdgeColor','r');
                hold on;plot(xx0,yy10,'linewidth',1.5,'MarkerEdgeColor','b');
                xlim([0,max(x(end),x0(end))]);
%                 ylim([0.8,1.6]);
                set(gcf, 'Position', size);
                set(gca, 'FontSize', 25);
                
                figure(200);plot(xx,yy2,'linewidth',1.5,'MarkerEdgeColor','r');
                hold on;plot(xx0,yy20,'linewidth',1.5,'MarkerEdgeColor','b');
                xlim([0,max(x(end),x0(end))]);
%                 ylim([0.8,1.6]);
                set(gcf, 'Position', size);
                set(gca, 'FontSize', 25);
                
                figure(300);plot(xx,yy3,'linewidth',1.5,'MarkerEdgeColor','r');
                hold on;plot(xx0,yy30,'linewidth',1.5,'MarkerEdgeColor','b');
                xlim([0,max(x(end),x0(end))]);
%                 ylim([0.8,1.6]);
                set(gcf, 'Position', size);
                set(gca, 'FontSize', 25);
                a=3
            end
            %%
            Smooth_cow = Smooth_cow + 1;
        end
    end
end