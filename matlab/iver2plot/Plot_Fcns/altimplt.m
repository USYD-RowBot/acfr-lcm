function altimplt()
%depth plot
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    2010.05.26      sbs         Added alt(lcm) plot; moved from altitude
%                                plot
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
    ii = find(iver_t.ref.Next_Depth > 0);
    Goal = nan(size(iver_t.ref.Next_Depth));
    Goal(ii) = iver_t.ref.Next_Depth(ii);
    
    plot(iver_t.elaptime, iver_t.alt.DTB_Height, 'b','LineWidth',2);
    plot(iver_t.elaptime, Goal, 'r','LineWidth',2);
    
    legend_str = strvcat(legend_str,'HFB (uvc)', 'Goal (uvc)');
end

% lcmlog
try
    nav_t = evalin('base','nav_t');
    NAV=1;
catch
    NAV=0;
end

if NAV
    plot(nav_t.RDI.elaptime, nav_t.RDI.altitude,'c','LineWidth',2);
    legend_str=strvcat(legend_str,'RDI (lcm)');
end

hold off;
title('Height from bottom (Altimeter) vs Time');
ylabel('Height from bottom [m]');
legend(legend_str);

