
% matlab_plot
hold on

ff = 0;

x0 = 0;
y0 = 0;
z0 = 0;

f = 0;

while 1
    
    pause(1)
    %%
    fid=fopen('/tmp/log.txt');
    s = textscan(fid, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f');
    
    x = s{1};
    y = s{2};
    z = s{3};
    
    t = min([length(x) length(y) length(z)]);
    
%     view(0,90)
    
    if f ~= 0
        delete(f);
        delete(g);
    end
        
    
    hold on
    f = plot3(x(1:t),y(1:t),z(1:t),'k');
    g = plot3(x(t),y(t),z(t),'go');
    axis equal;
    hold off
    fclose(fid);
    %     set(gca,'ZDir','reverse');
    drawnow
end
