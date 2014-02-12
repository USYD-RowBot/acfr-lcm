function altitudeplt()
%altitude plot

% read from:
% 1) ocean server
% 2) DVL

%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%                    rme         Created and written.
%    2008.07.17      ar          Modified for iver plot
%    2009.06.17      ak          Added lcm log plot
%    2009.07.13      cw          Modified for time match
%    2010.05.26      sbs         Commented out alt(lcm) plot and moved it
%                                to altimeter plot 

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
    bottom = iver_t.z - iver_t.alt.DTB_Height;
    Goal = depthgoal(iver_t);
    
    plot(iver_t.elaptime, iver_t.z,...
         iver_t.elaptime, bottom,...
         iver_t.elaptime, Goal,'r-.');
    legend_str = strvcat(legend_str,'DFS (uvc)','bottom (uvc)','goal (calc)');
end

% lcmlog
try
    nav_t = evalin('base','nav_t');
    NAV=1;
catch
    NAV=0;
end

hold off;
title('Depth and Seafloor vs Time');
ylabel('Depth [m]');
legend(legend_str);

