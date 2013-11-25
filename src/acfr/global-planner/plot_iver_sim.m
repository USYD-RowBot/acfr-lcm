clear all
matlab_plot
hold on

ff = 0;

x0 = 0;
y0 = 0;
z0 = 0;

num_dubins = 1;

f = 0;

while 1
    
    pause(2)
    %%
    fid=fopen('/tmp/log.txt');
    fid2=fopen('/tmp/log_nav.txt');
%     fid3=fopen('/tmp/log_waypoint.txt');
    fid3=fopen('/tmp/log_waypoint_now.txt');
    
    s = textscan(fid, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f');
    s2 = textscan(fid2, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f');
    s3 = textscan(fid3, '%f %f %f');
    
    fclose(fid);
%     fclose(fid2);
    fclose(fid3);
    
    x = s{1};
    y = s{2};
    z = s{3};
    
    x2 = s2{1};
    y2 = s2{2};
    z2 = s2{3};
    
    x3 = s3{1};
    y3 = s3{2};
    z3 = s3{3};
    
    t = min([length(x) length(y) length(z)]);
    t2 = min([length(x2) length(y2) length(z2)]);
    t3 = min([length(x3) length(y3) length(z3)]);
    %     view(0,90)
    
    if f ~= 0
        %         delete(f);
        delete(g);
        delete(f2);
        delete(g2);
    end
    
    
    matlab_plot_repeat
    
    hold on
%     f = plot3(x(1:t),y(1:t),z(1:t),'k');
    f2 = plot3(x2(1:t2),y2(1:t2),z2(1:t2),'b');
    %     f3 = plot3(x2(1:t2),y2(1:t2),z2(1:t2),'r.','MarkerSize',4);
    
    paths_shown = 0;
    path_save = zeros(3,50);
    path_save_counter = 1;
    path_counter = t3-1;
    color = 0;
%     while paths_shown < num_dubins
%         if x3(path_counter) == 0
%             paths_shown = paths_shown + 1;
%             plot3(path_save(1,1:path_save_counter-1),path_save(2,1:path_save_counter-1),path_save(3,1:path_save_counter-1),'Color',[1-color color color], 'Marker', 'o')
%             color = color + 1/(num_dubins*2);
%             path_save_counter = 1;
%         else
%             path_save(1:3,path_save_counter) = [x3(path_counter) y3(path_counter) z3(path_counter)]';
%             path_save_counter = path_save_counter + 1;
%         end
%         
%         path_counter = path_counter - 1;
%         if path_counter == 0
%            break; 
%         end
%                     
%     end

    while paths_shown < 1
        if x3(path_counter) == 0
            paths_shown = paths_shown + 1;
            plot3(path_save(1,1:path_save_counter-1),path_save(2,1:path_save_counter-1),path_save(3,1:path_save_counter-1),'Color',[1-color color color], 'Marker', 'o')
            color = color + 1/(num_dubins*2);
            path_save_counter = 1;
        else
            path_save(1:3,path_save_counter) = [x3(path_counter) y3(path_counter) z3(path_counter)]';
            path_save_counter = path_save_counter + 1;
        end
        
        path_counter = path_counter - 1;
        if path_counter == 0
           break; 
        end
                    
    end
    
    
%     g = plot3(x(t),y(t),z(t),'ko');
    g2 = plot3(x2(t2),y2(t2),z2(t2),'bo');
    axis equal;
    %view(90,90);
    hold off
    
    %set(gca,'XDir','reverse');
    drawnow
end
