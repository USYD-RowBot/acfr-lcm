
%  matlab_plot
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
    fid2=fopen('/tmp/log_nav.txt');
    s = textscan(fid, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f');
    s2 = textscan(fid2, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f');
    
    x = s{1};
    y = s{2};
    z = s{3};
    
    x2 = s2{1};
    y2 = s2{2};
    z2 = s2{3};
    
    t = min([length(x) length(y) length(z)]);
    t2 = min([length(x2) length(y2) length(z2)]);
%     view(0,90)
    
    if f ~= 0
%         delete(f);
        delete(g);
        delete(f2);
        delete(g2);
    end
        
    
    hold on
    f = plot3(x(1:t),y(1:t),z(1:t),'k');
%     f2 = plot3(x2(1:t2),y2(1:t2),z2(1:t2),'b');
    g = plot3(x(t),y(t),z(t),'ko');
%     g2 = plot3(x2(t2),y2(t2),z2(t2),'bo');
    axis equal;
    %view(90,90);
    hold off
    fclose(fid);
    fclose(fid2);
    %set(gca,'XDir','reverse');
    drawnow
end
