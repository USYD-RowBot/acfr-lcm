function depthplt()
% depth plot
% read from:
% 1) ocean server
% 2) desert start

%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%                    rme         Created and written.
%    2008.07.17      ar          Modified for iver plot
%    2009.06.17      ak          Added lcm log plot
%    2009.07.13      cw          Modified for time match
%    2010.05.26      sbs         Modified the goal plot

user = get(1,'UserData');

cla reset;
legend_str=''; 
hold on;

% uvclog
try
    iver_t = evalin('base','iver_t');
    IVER=1;
catch
    IVER=0;
end

if IVER
    Goal = depthgoal(iver_t);

    plot(iver_t.elaptime, iver_t.z,'b')
    plot(iver_t.elaptime, Goal,'r','LineWidth',2);
    plot(iver_t.elaptime, iver_t.ref.Depth_K_Goal,'g');
    legend_str=strvcat(legend_str,'DFS (uvc)','goal (calc)','k\_goal (uvc)');
end

% lcmlog
try
    nav_t = evalin('base','nav_t');
    NAV=1;
catch
    NAV=0;
end

if NAV
    plot(nav_t.DESERT_STAR.elaptime, -nav_t.DESERT_STAR.depth,'c','LineWidth',2);
    plot(nav_t.OS_COMPASS.elaptime, -nav_t.OS_COMPASS.depth,'m','LineWidth',2);
    legend_str=strvcat(legend_str,'dstar (lcm)','os\_compass (lcm)');
end

hold off;
title('Depth vs Time');
ylabel('Depth [m]');
legend(legend_str,-1);
