function temp_plt()
%temp plot

% read from:
% 1) ocean server compass
% 2) desert star

%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%                    rme         Created and written.
%    2008.07.17      ar          Modified for iver plot
%    2009.06.17      ak          Added lcm log plot
%    2009.07.13      cw          Modified for time match

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
    plot(iver_t.elaptime, iver_t.comp.Inside_Temp, 'b.','LineWidth',2);
    legend_str=strvcat(legend_str,'os (uvc)');
end

% lcmlog
try
    nav_t = evalin('base','nav_t');
    NAV=1;
catch
    NAV=0;
end

if NAV
    plot(nav_t.MICROSTRAIN.elaptime, nav_t.MICROSTRAIN.Temperature,'g.','LineWidth',2);
    legend_str=strvcat(legend_str,'ms (lcm)');
end

hold off;
title('Internal Temperature vs Time');
ylabel('Temperature [C]');
legend(legend_str);
