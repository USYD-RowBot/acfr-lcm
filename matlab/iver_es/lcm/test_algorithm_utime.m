function data = test_algorithm_utime (nav_t)
clc
if ~isfield(nav_t, 'ES.elaptime')
    R = [-0.7071, 0.7071, 0.0000
        0.7071, 0.7071, 0.0000;
        0.0000, 0.0000,-1.0000];
    nav_t.RDI.uvw = zeros(length(nav_t.RDI.btv),3);
    nav_t.RDI.dt = [0; diff(nav_t.RDI.utime)./1E6];
    for ii = 1:length(nav_t.RDI.btv)
        for kk = 1:3
            if isnan(nav_t.RDI.btv(ii,kk))
                nav_t.RDI.btv(ii,kk) = nav_t.RDI.btv(ii,kk-1);
            end
        end
        nav_t.RDI.uvw(ii,:) = (R*nav_t.RDI.btv(ii,1:3)')';
    end
    nav_t.RDI.xyz = cumsum((nav_t.RDI.uvw).*(nav_t.RDI.dt*[1 1 1]));
    nav_t.ES.utime = nav_t.TRITECH_ES.utime;
    nav_t.ES.elaptime = (nav_t.TRITECH_ES.utime - ...
        nav_t.TRITECH_ES.utime(1))/1E6;
    nav_t.ES.x        = interp1(nav_t.RDI.utime, nav_t.RDI.xyz(:,1), ...
        nav_t.ES.utime);
    nav_t.ES.y        = interp1(nav_t.RDI.utime, nav_t.RDI.xyz(:,2), ...
        nav_t.ES.utime);
    nav_t.ES.depth    = interp1(nav_t.DESERT_STAR.utime, ...
        nav_t.DESERT_STAR.depth, nav_t.ES.utime);
    nav_t.ES.dx       = [0; diff(nav_t.ES.x)];
    nav_t.ES.alt      = interp1(nav_t.RDI.utime, nav_t.RDI.altitude, ...
        nav_t.ES.utime);
    nav_t.ES.bottom_y = nav_t.ES.depth + nav_t.ES.alt;
    nav_t.ES.bottom_x = nav_t.ES.x + 5*sin(pi/6);
    nav_t.ES.pitch    = interp1(nav_t.MICROSTRAIN.utime, ...
        nav_t.MICROSTRAIN.sEuler(:,2), nav_t.ES.utime);
    nav_t.ES.range    = nav_t.TRITECH_ES.range;
    for ll = 1:length(nav_t.ES.elaptime)
        if nav_t.ES.range(ll) == 0
            nav_t.ES.range(ll) = 50;
        end
    end
    nav_t.ES.plot_rx  = zeros(length(nav_t.ES.elaptime),9);
    nav_t.ES.plot_ry  = zeros(length(nav_t.ES.elaptime),9);
    beam = (-3:3).*(pi/180);
    for ll = 1:length(nav_t.ES.elaptime)
        if nav_t.ES.range(ll) == 0
            nav_t.ES.range(ll) = 50;
        end
        nav_t.ES.plot_rx(ll,1) = nav_t.ES.x(ll) + ...
            0.29*cos(nav_t.ES.pitch(ll)) + 0.075*sin(nav_t.ES.pitch(ll));
        nav_t.ES.plot_ry(ll,1) = nav_t.ES.depth(ll) + ...
            0.29*sin(nav_t.ES.pitch(ll)) + 0.075*cos(nav_t.ES.pitch(ll));
        nav_t.ES.plot_rx(ll,2:8) = nav_t.ES.plot_rx(ll,1) + ...
            nav_t.ES.range(ll)*cos(beam+nav_t.ES.pitch(ll));
        nav_t.ES.plot_ry(ll,2:8) = nav_t.ES.plot_ry(ll,1) + ...
            nav_t.ES.range(ll)*sin(beam+nav_t.ES.pitch(ll));
        nav_t.ES.plot_rx(ll,9) = nav_t.ES.plot_rx(ll,1);
        nav_t.ES.plot_ry(ll,9) = nav_t.ES.plot_ry(ll,1);
    end
end
close all
figure(1)
plot(1:length(nav_t.ES.depth), nav_t.ES.depth, ...
    1:length(nav_t.ES.bottom_y), nav_t.ES.bottom_y)
set(gca,'YDir','reverse')
dummy = ginput(2);
nav_t.ES.h.start = ceil(dummy(1,1));
nav_t.ES.h.stop  = ceil(dummy(2,1));
if nav_t.ES.h.start < 32
    nav_t.ES.h.start = 32;
end
if nav_t.ES.h.stop > length(nav_t.ES.elaptime)-151
    nav_t.ES.h.stop = length(nav_t.ES.elaptime)-151;
end
if nav_t.ES.h.start >= nav_t.ES.h.stop
    error('Sim Limit Error! Sim Start > Sim Stop')
end
close all
nav_t.ES.h.depth       = zeros(1,50);
nav_t.ES.h.health      = -ones(1,50);
nav_t.ES.h.index       = 0;
nav_t.ES.h.offset      = 1;
nav_t.ES.h.slope       = 0;
nav_t.ES.h.flagSlope   = -1;
nav_t.ES.h.maxSlope = tand(25/2);
nav_t.ES.h.minHeight = 3;
nav_t.ES.h_save.depth     = zeros(length(nav_t.ES.h.start:nav_t.ES.h.stop),50);
nav_t.ES.h_save.health    = zeros(length(nav_t.ES.h.start:nav_t.ES.h.stop),50);
nav_t.ES.h_save.index     = zeros(length(nav_t.ES.h.start:nav_t.ES.h.stop),1);
nav_t.ES.h_save.offset    = zeros(length(nav_t.ES.h.start:nav_t.ES.h.stop),1);
nav_t.ES.h_save.slope     = zeros(length(nav_t.ES.h.start:nav_t.ES.h.stop),1);
nav_t.ES.h_save.flagSlope = zeros(length(nav_t.ES.h.start:nav_t.ES.h.stop),1);
count = 1;
aviobj = avifile('whoi.avi');
fig = figure;
for jj = nav_t.ES.h.start:nav_t.ES.h.stop
    nav_t = updateHorizon(nav_t, jj);
    plot(nav_t.ES.x(jj), nav_t.ES.depth(jj), 'rx')
    hold on
%     hx = (0:49) + (nav_t.ES.x(jj) + nav_t.ES.h.offset)*ones(1,50);
%     scatter(hx, nav_t.ES.h.depth, 16, nav_t.ES.h.health)
%     colormap([1 1 1; 1 1 0; 0 1 0]);
%     caxis([-1 1])
%     if nav_t.ES.h.index > 0
%         plot(hx(nav_t.ES.h.index), nav_t.ES.h.depth(nav_t.ES.h.index), 'bx')
%     end
%     if nav_t.ES.h.flagSlope > tand(15)
%         slopeX(1) = nav_t.ES.plot_rx(jj,1) + nav_t.ES.h.index + ...
%             nav_t.ES.h.offset;
%         slopeX(2) = slopeX(1) - 50;
%         slopeY(1) = nav_t.ES.h.depth(nav_t.ES.h.index);
%         slopeY(2) = slopeY(1) - diff(slopeX)*tand(15);
%         plot(slopeX, slopeY, 'r')
%     end
    if nav_t.ES.range(jj) < 5
        plot(nav_t.ES.plot_rx(jj,:), nav_t.ES.plot_ry(jj,:), 'y')
    else
        plot(nav_t.ES.plot_rx(jj,:), nav_t.ES.plot_ry(jj,:), 'b')
    end
%     plot(nav_t.ES.x(jj), nav_t.ES.depth(jj) + 3, 'ro')
    plot(nav_t.ES.bottom_x(jj-50:jj+150), ...
        nav_t.ES.bottom_y(jj-50:jj+150), 'k')
    xlim(nav_t.ES.x(jj) + [-10 55])
    ylim([-2 max(nav_t.ES.bottom_y)+5])
    set(gca,'YDir','reverse')
    xlabel('Distance along Trackline [m]')
    ylabel('Depth [m]')
%     title(['Slope ', num2str(nav_t.ES.h.flagSlope), ...
%         ', Max Slope ', num2str(tand(15)), ...
%         ',  Pitch ', num2str(nav_t.ES.pitch(jj))])
    hold off
    F = getframe(fig);
    aviobj = addframe(aviobj, F);
    pause(abs(nav_t.ES.dx(jj))/50)
    
    nav_t.ES.h_save.depth(count,:)   = nav_t.ES.h.depth;
    nav_t.ES.h_save.health(count,:)  = nav_t.ES.h.health;
    nav_t.ES.h_save.index(count)     = nav_t.ES.h.index;
    nav_t.ES.h_save.offset(count)    = nav_t.ES.h.offset;
    nav_t.ES.h_save.slope(count)     = nav_t.ES.h.slope;
    nav_t.ES.h_save.flagSlope(count) = nav_t.ES.h.flagSlope;
    count = count + 1;
end
close (fig)
aviobj = close(aviobj);
data=nav_t;
end